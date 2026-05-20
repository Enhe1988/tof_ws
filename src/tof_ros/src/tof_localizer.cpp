#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <cmath>
#include <deque>

// ── Globals ───────────────────────────────────────────────────────────────────
static pcl::PointCloud<pcl::PointXYZ>::Ptr g_map;
static ros::Publisher                       g_pose_pub;
static ros::Publisher                       g_aligned_pub;
static tf2_ros::TransformBroadcaster*       g_tf_br;
static Eigen::Matrix4f                      g_T = Eigen::Matrix4f::Identity();
static Eigen::Matrix4f                      g_T_init = Eigen::Matrix4f::Identity();
static double                               g_max_corr;
static double                               g_max_fitness;
static double                               g_max_jump;
static double                               g_src_voxel;
static std::string                          g_sensor_frame;
static int                                  g_accum_frames;
static int                                  g_fail_count  = 0;
static int                                  g_max_fails   = 30; // 连续失败 N 帧后重置到初始猜测
static std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> g_cloud_buf;

// ── Build initial 4×4 guess ───────────────────────────────────────────────────
// use_r_mount=true  → tof_frame（z前/x右/y下）→ map，需要 R_mount 转换
// use_r_mount=false → 传感器坐标系与地图一致（x前/y左/z上），只做 heading 旋转
// heading_deg: 传感器前向在地图中的朝向（以+x_map为0°，逆时针为正）
static Eigen::Matrix4f makeInitialGuess(double x, double y, double z,
                                        double heading_deg, bool use_r_mount)
{
    float h = static_cast<float>(heading_deg * M_PI / 180.0);
    Eigen::Matrix3f R;

    if (use_r_mount) {
        static const Eigen::Matrix3f R_mount =
            (Eigen::AngleAxisf(static_cast<float>(-M_PI / 2), Eigen::Vector3f::UnitZ())
           * Eigen::AngleAxisf(static_cast<float>(-M_PI / 2), Eigen::Vector3f::UnitX()))
            .toRotationMatrix();
        R = Eigen::AngleAxisf(h, Eigen::Vector3f::UnitZ()).toRotationMatrix() * R_mount;
    } else {
        R = Eigen::AngleAxisf(h, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    }

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = R;
    T(0, 3) = static_cast<float>(x);
    T(1, 3) = static_cast<float>(y);
    T(2, 3) = static_cast<float>(z);
    return T;
}

// ── Callback ──────────────────────────────────────────────────────────────────
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *frame);

    // ── 多帧累积 ──────────────────────────────────────────────────────────────
    g_cloud_buf.push_back(frame);
    if (static_cast<int>(g_cloud_buf.size()) > g_accum_frames)
        g_cloud_buf.pop_front();
    if (static_cast<int>(g_cloud_buf.size()) < g_accum_frames)
        return;  // 缓冲区还没满，等待更多帧

    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& c : g_cloud_buf)
        *src += *c;

    // ── source 点云下采样（累积后去重复点） ──────────────────────────────────
    if (g_src_voxel > 0.0) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(src);
        vg.setLeafSize(static_cast<float>(g_src_voxel),
                       static_cast<float>(g_src_voxel),
                       static_cast<float>(g_src_voxel));
        vg.filter(*filtered);
        src = filtered;
    }

    if (static_cast<int>(src->size()) < 20) {
        ROS_WARN_THROTTLE(2.0, "Source cloud too sparse (%zu pts), skipping", src->size());
        return;
    }
    ROS_INFO_THROTTLE(1.0, "src pts: %zu", src->size());

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(src);
    gicp.setInputTarget(g_map);
    gicp.setMaxCorrespondenceDistance(g_max_corr);
    gicp.setMaximumIterations(50);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
    auto t0 = ros::WallTime::now();
    gicp.align(*aligned, g_T);
    double gicp_ms = (ros::WallTime::now() - t0).toSec() * 1000.0;
    ROS_DEBUG_THROTTLE(1.0, "GICP took %.1f ms", gicp_ms);

    auto reject = [&](const char* reason) {
        if (++g_fail_count >= g_max_fails) {
            ROS_WARN("连续 %d 帧定位失败，重置到初始猜测", g_fail_count);
            g_T = g_T_init;
            g_fail_count = 0;
            g_cloud_buf.clear();
        }
        ROS_WARN_THROTTLE(1.0, "%s", reason);
    };

    if (!gicp.hasConverged()) {
        reject("GICP did not converge");
        return;
    }

    float fitness = gicp.getFitnessScore();
    Eigen::Matrix4f T_new = gicp.getFinalTransformation();

    // ── 防跳变检查 ────────────────────────────────────────────────────────────
    if (fitness > static_cast<float>(g_max_fitness)) {
        char buf[128];
        snprintf(buf, sizeof(buf), "fitness=%.4f 超过阈值 %.4f，拒绝更新", fitness, g_max_fitness);
        reject(buf);
        return;
    }
    float jump = (T_new.block<3,1>(0,3) - g_T.block<3,1>(0,3)).norm();
    if (jump > static_cast<float>(g_max_jump)) {
        char buf[128];
        snprintf(buf, sizeof(buf), "帧间跳变 %.3f m 超过限制 %.3f m，拒绝更新", jump, g_max_jump);
        reject(buf);
        return;
    }

    g_fail_count = 0;
    g_T = T_new;

    Eigen::Vector3f    t = g_T.block<3, 1>(0, 3);
    Eigen::Quaternionf q(g_T.block<3, 3>(0, 0));

    // 传感器光轴（tof_z）在地图 XY 平面的朝向角（与 init_heading_deg 含义相同）
    Eigen::Vector3f tof_fwd = g_T.block<3, 3>(0, 0).col(2);
    float heading_out = std::atan2(tof_fwd(1), tof_fwd(0)) * 180.f / M_PI;
    ROS_INFO_THROTTLE(1.0,
        "GICP converged: fitness=%.4f  pos=(%.3f, %.3f, %.3f)  heading=%.1f deg",
        fitness, t(0), t(1), t(2), heading_out);

    ros::Time stamp = msg->header.stamp;

    // ── Pose ─────────────────────────────────────────────────────────────────
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp       = stamp;
    pose_msg.header.frame_id    = "map";
    pose_msg.pose.position.x    = t(0);
    pose_msg.pose.position.y    = t(1);
    pose_msg.pose.position.z    = t(2);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    g_pose_pub.publish(pose_msg);

    // ── TF: map → tof_frame ───────────────────────────────────────────────────
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp            = stamp;
    tf_msg.header.frame_id         = "map";
    tf_msg.child_frame_id          = g_sensor_frame;
    tf_msg.transform.translation.x = t(0);
    tf_msg.transform.translation.y = t(1);
    tf_msg.transform.translation.z = t(2);
    tf_msg.transform.rotation.x    = q.x();
    tf_msg.transform.rotation.y    = q.y();
    tf_msg.transform.rotation.z    = q.z();
    tf_msg.transform.rotation.w    = q.w();
    g_tf_br->sendTransform(tf_msg);

    // ── Aligned cloud (in map frame) ──────────────────────────────────────────
    sensor_msgs::PointCloud2 aligned_msg;
    pcl::toROSMsg(*aligned, aligned_msg);
    aligned_msg.header.stamp    = stamp;
    aligned_msg.header.frame_id = "map";
    g_aligned_pub.publish(aligned_msg);
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tof_localizer");
    ros::NodeHandle nh("~");

    std::string map_path, input_topic;
    double voxel_size, init_x, init_y, init_z, init_heading_deg;
    bool use_r_mount;

    nh.param<std::string>("map_path",      map_path,      "");
    nh.param<std::string>("input_topic",   input_topic,   "/tof/points");
    nh.param<std::string>("sensor_frame",  g_sensor_frame, "tof_frame");
    nh.param("voxel_size",       voxel_size,       0.10);
    nh.param("src_voxel_size",   g_src_voxel,      0.0);
    nh.param("accumulate_frames", g_accum_frames,  1);
    nh.param("use_r_mount",      use_r_mount,      true);
    nh.param("init_x",           init_x,           0.0);
    nh.param("init_y",           init_y,           0.0);
    nh.param("init_z",           init_z,           1.0);
    nh.param("init_heading_deg", init_heading_deg, 0.0);
    nh.param("max_corr_dist",    g_max_corr,       1.0);
    nh.param("max_fitness",      g_max_fitness,    0.1);
    nh.param("max_jump",         g_max_jump,       0.3);

    if (map_path.empty()) {
        ROS_FATAL("~map_path param is required");
        return 1;
    }

    // ── Load prior map ────────────────────────────────────────────────────────
    ROS_INFO("Loading prior map: %s", map_path.c_str());
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *raw) < 0) {
        ROS_FATAL("Failed to load map: %s", map_path.c_str());
        return 1;
    }
    ROS_INFO("Map loaded: %zu pts", raw->size());

    // ── Downsample ────────────────────────────────────────────────────────────
    g_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(raw);
    vg.setLeafSize(static_cast<float>(voxel_size),
                   static_cast<float>(voxel_size),
                   static_cast<float>(voxel_size));
    vg.filter(*g_map);
    ROS_INFO("Map downsampled to %zu pts (leaf=%.2f m, max_corr=%.2f m)",
             g_map->size(), voxel_size, g_max_corr);

    // ── Initial pose guess ────────────────────────────────────────────────────
    g_T      = makeInitialGuess(init_x, init_y, init_z, init_heading_deg, use_r_mount);
    g_T_init = g_T;
    ROS_INFO("Initial guess: pos=(%.2f, %.2f, %.2f) heading=%.1f deg",
             init_x, init_y, init_z, init_heading_deg);

    // ── Publishers ────────────────────────────────────────────────────────────
    ros::NodeHandle nh_pub;
    g_pose_pub    = nh_pub.advertise<geometry_msgs::PoseStamped>("/tof/pose",    10);
    g_aligned_pub = nh_pub.advertise<sensor_msgs::PointCloud2>  ("/tof/aligned", 10);

    // Latched map cloud for RViz
    auto map_pub = nh_pub.advertise<sensor_msgs::PointCloud2>("/tof/map", 1, /*latch=*/true);
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*g_map, map_msg);
    map_msg.header.stamp    = ros::Time::now();
    map_msg.header.frame_id = "map";
    map_pub.publish(map_msg);
    ROS_INFO("Prior map published on /tof/map (latched)");

    tf2_ros::TransformBroadcaster tf_br;
    g_tf_br = &tf_br;

    // Broadcast initial guess TF immediately so RViz can display the live cloud
    {
        Eigen::Vector3f    t0 = g_T.block<3, 1>(0, 3);
        Eigen::Quaternionf q0(g_T.block<3, 3>(0, 0));
        geometry_msgs::TransformStamped init_tf;
        init_tf.header.stamp            = ros::Time::now();
        init_tf.header.frame_id         = "map";
        init_tf.child_frame_id          = g_sensor_frame;
        init_tf.transform.translation.x = t0(0);
        init_tf.transform.translation.y = t0(1);
        init_tf.transform.translation.z = t0(2);
        init_tf.transform.rotation.x    = q0.x();
        init_tf.transform.rotation.y    = q0.y();
        init_tf.transform.rotation.z    = q0.z();
        init_tf.transform.rotation.w    = q0.w();
        tf_br.sendTransform(init_tf);
    }

    auto sub = nh_pub.subscribe(input_topic, 10, cloudCallback);

    ROS_INFO("tof_localizer ready — waiting for %s", input_topic.c_str());
    ros::spin();
    return 0;
}
