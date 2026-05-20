#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <climits>
#include <limits>

// ── Protocol constants ────────────────────────────────────────────────────────
static constexpr uint32_t MAGIC_EXPECTED   = 0x544F4600U;  // "TOF\0" little-endian
static constexpr uint16_t VERSION_EXPECTED = 2U;
static constexpr int      UDP_PORT         = 55320;
static constexpr int      MAX_UDP_BUF      = 65535;

// ── TMF8829 optical FOV (from datasheet §5, p.15) ────────────────────────────
static constexpr double FOV_X_RAD = 67.9 * M_PI / 180.0;  // horizontal
static constexpr double FOV_Y_RAD = 52.8 * M_PI / 180.0;  // vertical

// ── Sub-capture zone layout ───────────────────────────────────────────────────
// TMF8829 48×32 mode uses two sequential sub-captures. Each sub-capture covers
// the full vertical FOV. In the payload they are stored as two contiguous halves:
//   data rows  0 .. (rows/2 - 1)  → sub-cap 0 → even spatial rows 0, 2, 4, …
//   data rows (rows/2) .. (rows-1) → sub-cap 1 → odd  spatial rows 1, 3, 5, …
// This remapping corrects the zone-to-angle mapping.
static uint16_t data_row_to_spatial(uint16_t data_row, uint16_t rows)
{
    uint16_t half = rows / 2;
    return (data_row < half) ? data_row * 2
                             : (data_row - half) * 2 + 1;
}

// ── Packed structs matching the UDP frame layout (little-endian) ──────────────
#pragma pack(push, 1)
struct TofUdpHeader {
    uint32_t magic;         // 0x544F4600
    uint16_t version;       // 2
    uint16_t mode;          // 0x04 for 48×32
    uint32_t frame_id;      // monotonically increasing
    uint64_t timestamp_us;  // CLOCK_MONOTONIC µs
    uint16_t cols;          // 48
    uint16_t rows;          // 32
    uint32_t payload_bytes; // cols * rows * 3
};

struct TofZone {
    uint16_t dist_mm;  // 0 = invalid
    uint8_t  conf;     // 0 = invalid
};
#pragma pack(pop)

static_assert(sizeof(TofUdpHeader) == 28, "TofUdpHeader must be 28 bytes");
static_assert(sizeof(TofZone)      == 3,  "TofZone must be 3 bytes");

// ── Helpers ───────────────────────────────────────────────────────────────────

// Convert zone (spatial_row, col) + radial distance to 3-D point in tof_frame.
// spatial_row is the remapped row (0-31 after sub-capture deinterleaving).
// Frame convention: z forward (optical axis), x right, y down.
// Spherical model: x²+y²+z² = dist² guaranteed.
static void zone_to_xyz(uint16_t col, uint16_t cols,
                        uint16_t spatial_row, uint16_t rows,
                        float dist_m,
                        float& x, float& y, float& z)
{
    float angle_h = ((col         + 0.5f) / cols - 0.5f) * static_cast<float>(FOV_X_RAD);
    float angle_v = ((spatial_row + 0.5f) / rows - 0.5f) * static_cast<float>(FOV_Y_RAD);

    x = -dist_m * std::sin(angle_h) * std::cos(angle_v);
    y = dist_m * std::sin(angle_v);
    z = dist_m * std::cos(angle_h) * std::cos(angle_v);
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tof_receiver");
    ros::NodeHandle nh("~");

    // ── Tunable parameters ────────────────────────────────────────────────────
    int    conf_threshold;   // minimum confidence (0-255) to accept a zone
    double dist_min_m;       // minimum valid distance in metres (cuts nearby obstructions)
    double dist_max_m;       // maximum valid distance in metres
    int    sor_neighbors;    // StatisticalOutlierRemoval: k neighbours
    double sor_std_ratio;    // StatisticalOutlierRemoval: std-dev multiplier

    nh.param("conf_threshold", conf_threshold, 30);
    nh.param("dist_min_m",     dist_min_m,     0.05);
    nh.param("dist_max_m",     dist_max_m,     5.0);
    nh.param("sor_neighbors",  sor_neighbors,  10);
    nh.param("sor_std_ratio",  sor_std_ratio,  2.0);

    ros::NodeHandle nh_pub;
    auto pc_pub  = nh_pub.advertise<sensor_msgs::PointCloud2>("/tof/points", 10);
    auto img_pub = nh_pub.advertise<sensor_msgs::Image>("/tof/depth",  10);

    // ── UDP socket setup ──────────────────────────────────────────────────────
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ROS_FATAL("socket(): %s", strerror(errno));
        return 1;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Non-blocking recv via timeout so the loop can respond to Ctrl-C
    struct timeval tv{ 0, 100'000 };  // 100 ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(UDP_PORT);

    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ROS_FATAL("bind() on port %d: %s", UDP_PORT, strerror(errno));
        close(sock);
        return 1;
    }

    ROS_INFO("tof_receiver listening on UDP 0.0.0.0:%d", UDP_PORT);
    ROS_INFO("conf_threshold=%d  dist=[%.2f, %.1f]m  sor_k=%d  sor_std=%.1f",
             conf_threshold, dist_min_m, dist_max_m, sor_neighbors, sor_std_ratio);

    // ── Receive loop ──────────────────────────────────────────────────────────
    static uint8_t buf[MAX_UDP_BUF];
    uint32_t last_frame_id = 0;
    bool     first_frame   = true;

    while (ros::ok()) {
        ssize_t n = recv(sock, buf, sizeof(buf), 0);

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;  // timeout – check ros::ok()
            ROS_WARN_THROTTLE(5.0, "recv(): %s", strerror(errno));
            continue;
        }

        // ── Header validation ─────────────────────────────────────────────────
        if (static_cast<size_t>(n) < sizeof(TofUdpHeader)) {
            ROS_WARN("Packet too short (%zd bytes), discarding", n);
            continue;
        }

        TofUdpHeader hdr;
        std::memcpy(&hdr, buf, sizeof(TofUdpHeader));

        if (hdr.magic != MAGIC_EXPECTED) {
            ROS_WARN("Bad magic 0x%08X (expected 0x%08X), discarding",
                     hdr.magic, MAGIC_EXPECTED);
            continue;
        }
        if (hdr.version != VERSION_EXPECTED) {
            ROS_WARN("Bad version %u (expected %u), discarding",
                     hdr.version, VERSION_EXPECTED);
            continue;
        }

        const uint32_t zone_count    = static_cast<uint32_t>(hdr.cols) * hdr.rows;
        const uint32_t expected_pay  = zone_count * sizeof(TofZone);

        if (hdr.payload_bytes != expected_pay) {
            ROS_WARN("payload_bytes mismatch: hdr says %u but cols×rows×3=%u",
                     hdr.payload_bytes, expected_pay);
            continue;
        }
        if (static_cast<size_t>(n) < sizeof(TofUdpHeader) + hdr.payload_bytes) {
            ROS_WARN("Packet truncated (%zd bytes), discarding", n);
            continue;
        }

        // ── Frame-drop detection ──────────────────────────────────────────────
        if (!first_frame && hdr.frame_id != last_frame_id + 1) {
            uint32_t dropped = hdr.frame_id - (last_frame_id + 1);
            ROS_WARN("Frame drop! expected frame_id=%u, got=%u, ~%u frame(s) lost",
                     last_frame_id + 1, hdr.frame_id, dropped);
        }
        first_frame   = false;
        last_frame_id = hdr.frame_id;

        // ── Parse zones ───────────────────────────────────────────────────────
        const uint16_t cols  = hdr.cols;
        const uint16_t rows  = hdr.rows;
        const TofZone* zones = reinterpret_cast<const TofZone*>(buf + sizeof(TofUdpHeader));

        // Timestamp: sensor provides CLOCK_MONOTONIC µs → ros::Time via nsec
        ros::Time stamp;
        stamp.fromNSec(hdr.timestamp_us * 1000ULL);

        // ── Step 1: filter by confidence + distance, project to 3-D ─────────
        const float dist_min_f = static_cast<float>(dist_min_m);
        const float dist_max_f = static_cast<float>(dist_max_m);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->reserve(zone_count);

        uint32_t n_invalid = 0, n_low_conf = 0, n_dist_range = 0;
        for (uint16_t row = 0; row < rows; ++row) {
            uint16_t srow = data_row_to_spatial(row, rows);
            for (uint16_t col = 0; col < cols; ++col) {
                const TofZone& z = zones[row * cols + col];
                if (z.dist_mm == 0) { ++n_invalid; continue; }
                if (z.conf < static_cast<uint8_t>(conf_threshold)) { ++n_low_conf; continue; }
                // dist_mm 字段单位为 0.25 mm/LSB（数据手册 §8.2.37）
                float dist_m = static_cast<float>(z.dist_mm) * 0.25f * 1e-3f;
                if (dist_m < dist_min_f || dist_m > dist_max_f) { ++n_dist_range; continue; }

                float px, py, pz;
                zone_to_xyz(col, cols, srow, rows, dist_m, px, py, pz);
                cloud->push_back({px, py, pz});
            }
        }
        ROS_INFO_THROTTLE(2.0,
            "zones=%u  invalid=%u  low_conf(<%d)=%u  dist_range=%u  passed=%zu",
            zone_count, n_invalid, conf_threshold, n_low_conf, n_dist_range, cloud->size());

        // ── Step 2: statistical outlier removal ───────────────────────────────
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        if (static_cast<int>(cloud->size()) > sor_neighbors) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud);
            sor.setMeanK(sor_neighbors);
            sor.setStddevMulThresh(sor_std_ratio);
            sor.filter(*cloud_filtered);
        } else {
            cloud_filtered = cloud;
        }

        // ── Step 3: convert to ROS PointCloud2 ───────────────────────────────
        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(*cloud_filtered, pc_msg);
        pc_msg.header.stamp    = stamp;
        pc_msg.header.frame_id = "tof_frame";

        // ── Depth Image (32FC1, metres, NaN for invalid) ──────────────────────
        sensor_msgs::Image img_msg;
        img_msg.header.stamp    = stamp;
        img_msg.header.frame_id = "tof_frame";
        img_msg.height          = rows;
        img_msg.width           = cols;
        img_msg.encoding        = sensor_msgs::image_encodings::TYPE_32FC1;
        img_msg.is_bigendian    = 0;
        img_msg.step            = cols * sizeof(float);
        img_msg.data.resize(static_cast<size_t>(rows) * cols * sizeof(float));

        float* pixels = reinterpret_cast<float*>(img_msg.data.data());
        const float kNaN = std::numeric_limits<float>::quiet_NaN();

        for (uint16_t row = 0; row < rows; ++row) {
            uint16_t srow = data_row_to_spatial(row, rows);
            for (uint16_t col = 0; col < cols; ++col) {
                const TofZone& z = zones[row * cols + col];
                pixels[srow * cols + col] = (z.dist_mm > 0 && z.conf > 0)
                                            ? static_cast<float>(z.dist_mm) * 0.25f * 1e-3f
                                            : kNaN;
            }
        }

        pc_pub.publish(pc_msg);
        img_pub.publish(img_msg);

        ros::spinOnce();
    }

    close(sock);
    return 0;
}
