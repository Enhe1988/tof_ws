#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>

// ── Upixels 出厂协议常量（手册 §5.1.1，14字节帧）──────────────────────────────
static constexpr uint8_t FRAME_HEADER     = 0xFE;
static constexpr uint8_t FRAME_BYTE_COUNT = 0x0A;  // 数据段固定10字节
static constexpr uint8_t FRAME_TAIL       = 0x55;
static constexpr uint8_t FLOW_VALID       = 0xF5;  // valid字段：光流有效
static constexpr int     FRAME_PAYLOAD    = 13;    // 帧头之后的字节数

// ── 共享状态（串口线程写，ROS回调线程读）────────────────────────────────────────
struct State {
    double pos_x      = 0.0;
    double pos_y      = 0.0;
    double pos_z      = 0.0;  // 实时激光对地高度
    double heading    = 0.0;  // 来自 /tof/pose 的最新 yaw（rad）
    std::mutex mtx;
};
static State          g_st;
static std::atomic<bool> g_running{true};

// ── /tof/pose 回调：修正积分基点 + 更新朝向 ─────────────────────────────────
// heading 取 tof_z 轴在 map XY 的投影角（与 tof_localizer heading 定义一致）
static void tofPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    const double qx = msg->pose.orientation.x;
    const double qy = msg->pose.orientation.y;
    const double qz = msg->pose.orientation.z;
    const double qw = msg->pose.orientation.w;
    // R * e_z = [2(qx*qz + qw*qy),  2(qy*qz - qw*qx),  ...]
    const double fwd_x = 2.0 * (qx * qz + qw * qy);
    const double fwd_y = 2.0 * (qy * qz - qw * qx);

    std::lock_guard<std::mutex> lk(g_st.mtx);
    g_st.pos_x   = msg->pose.position.x;
    g_st.pos_y   = msg->pose.position.y;
    g_st.heading = std::atan2(fwd_y, fwd_x);
    ROS_DEBUG("光流基点修正: (%.3f, %.3f) heading=%.1f°",
              g_st.pos_x, g_st.pos_y, g_st.heading * 180.0 / M_PI);
}

// ── 打开串口（POSIX termios，115200 8N1）────────────────────────────────────────
static int openSerial(const std::string& port)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        ROS_FATAL("无法打开串口 %s: %s", port.c_str(), strerror(errno));
        return -1;
    }
    struct termios tty{};
    if (tcgetattr(fd, &tty) < 0) {
        ROS_FATAL("tcgetattr: %s", strerror(errno));
        close(fd); return -1;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 10;  // 1s 超时，配合 ros::ok() 检查
    if (tcsetattr(fd, TCSANOW, &tty) < 0) {
        ROS_FATAL("tcsetattr: %s", strerror(errno));
        close(fd); return -1;
    }
    tcflush(fd, TCIFLUSH);
    return fd;
}

// ── 读取精确 n 字节，失败/超时返回 false ──────────────────────────────────────
static bool readExact(int fd, uint8_t* buf, int n)
{
    int got = 0;
    while (got < n && g_running.load()) {
        ssize_t r = read(fd, buf + got, n - got);
        if (r < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (r == 0) return false;
        got += static_cast<int>(r);
    }
    return (got == n);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "oflow_odometry");
    ros::NodeHandle nh("~");

    // ── 参数 ──────────────────────────────────────────────────────────────────
    std::string port;
    double mount_yaw_deg;    // 光流模块机头(Y轴)相对于ToF传感器前向的安装偏角（°）
    double height_override;  // >0：强制使用固定高度(m)；<0：使用激光测距
    double init_heading_deg; // 无ToF数据时的初始朝向假设（°）
    int    flow_sign_x;      // 光流X符号，若实测方向相反改为 -1
    int    flow_sign_y;      // 光流Y符号
    int    min_laser_conf;   // 激光测距最低置信度（0x64=100%）

    nh.param<std::string>("port",             port,             "/dev/ttyUSB0");
    nh.param("mount_yaw_deg",    mount_yaw_deg,    0.0);
    nh.param("height_override",  height_override,  -1.0);
    nh.param("init_heading_deg", init_heading_deg, 0.0);
    nh.param("flow_sign_x",      flow_sign_x,      1);
    nh.param("flow_sign_y",      flow_sign_y,      1);
    nh.param("min_laser_conf",   min_laser_conf,   10);

    double mount_yaw_rad = mount_yaw_deg * M_PI / 180.0;
    {
        std::lock_guard<std::mutex> lk(g_st.mtx);
        g_st.heading = init_heading_deg * M_PI / 180.0;
    }

    int fd = openSerial(port);
    if (fd < 0) return 1;

    // ── ROS 接口 ──────────────────────────────────────────────────────────────
    ros::NodeHandle nh_pub;
    auto pose_pub = nh_pub.advertise<geometry_msgs::PoseStamped>("/oflow/pose", 10);
    auto path_pub = nh_pub.advertise<nav_msgs::Path>("/oflow/path", 10);

    // /tof/pose 回调：ToF定位成功时修正积分基点 + 更新朝向
    auto tof_sub = nh_pub.subscribe("/tof/pose", 5, tofPoseCallback);

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";

    // ── 串口解析线程 ──────────────────────────────────────────────────────────
    std::thread serial_thread([&]() {
        ROS_INFO("oflow_odometry: 串口 %s 已打开，等待数据...", port.c_str());

        uint8_t sync;
        uint8_t rest[FRAME_PAYLOAD];  // 帧头之后的13字节

        while (g_running.load() && ros::ok()) {

            // 1. 对齐帧头 0xFE
            if (!readExact(fd, &sync, 1)) continue;
            if (sync != FRAME_HEADER)    continue;

            // 2. 读剩余 13 字节
            if (!readExact(fd, rest, FRAME_PAYLOAD)) continue;

            // 3. 基础格式校验
            if (rest[0] != FRAME_BYTE_COUNT) continue;  // 字节数字段
            if (rest[12] != FRAME_TAIL)      continue;  // 包尾

            // 4. XOR 校验（手册：字节3-12 即 rest[1..10]）
            uint8_t xor_calc = 0;
            for (int i = 1; i <= 10; ++i) xor_calc ^= rest[i];
            if (xor_calc != rest[11]) {
                ROS_WARN_THROTTLE(2.0, "XOR 校验失败: 计算=0x%02X 期望=0x%02X",
                                  xor_calc, rest[11]);
                continue;
            }

            // 5. 解析字段（小端序）
            const int16_t  flow_x   = static_cast<int16_t>( rest[1] | (rest[2]  << 8));
            const int16_t  flow_y   = static_cast<int16_t>( rest[3] | (rest[4]  << 8));
            const uint16_t dt_us    = static_cast<uint16_t>(rest[5] | (rest[6]  << 8));
            const uint16_t dist_mm  = static_cast<uint16_t>(rest[7] | (rest[8]  << 8));
            const uint8_t  valid    = rest[9];
            const uint8_t  lconf    = rest[10];

            // 6. 光流有效性
            if (valid != FLOW_VALID) {
                ROS_WARN_THROTTLE(2.0, "光流数据无效 (valid=0x%02X)", valid);
                continue;
            }

            // 7. 高度确定
            double height_m;
            if (height_override > 0.0) {
                height_m = height_override;
            } else if (dist_mm == 0 || dist_mm == 0xFFFF) {
                ROS_WARN_THROTTLE(2.0, "激光测距超量程 dist=0x%04X，丢弃", dist_mm);
                continue;
            } else if (static_cast<int>(lconf) < min_laser_conf) {
                ROS_WARN_THROTTLE(2.0, "激光置信度过低 %u < %d，丢弃", lconf, min_laser_conf);
                continue;
            } else {
                height_m = dist_mm * 1e-3;
            }

            // 8. 角位移 → 传感器坐标系线位移
            //    flow_x_integral 对应传感器 X 轴（机身右方向）
            //    flow_y_integral 对应传感器 Y 轴（机头前方向）
            //    换算公式：实际位移(m) = integral / 10000 × height_m
            //    注：sign 参数用于初次调试时修正方向，正常情况均为 1
            const double dx_s = flow_sign_x * (flow_x / 10000.0) * height_m;
            const double dy_s = flow_sign_y * (flow_y / 10000.0) * height_m;

            // 9. 传感器坐标系 → map 坐标系
            //    θ = 传感器机头(Y轴)在 map 中的朝向角
            //    传感器 X 单位向量（右）在 map = [ sin(θ), -cos(θ)]
            //    传感器 Y 单位向量（前）在 map = [ cos(θ),  sin(θ)]
            double heading_now;
            {
                std::lock_guard<std::mutex> lk(g_st.mtx);
                heading_now = g_st.heading;
            }
            const double theta  = heading_now + mount_yaw_rad;
            const double dx_map = dx_s * std::sin(theta) + dy_s * std::cos(theta);
            const double dy_map = -dx_s * std::cos(theta) + dy_s * std::sin(theta);

            // 10. 积分并发布
            geometry_msgs::PoseStamped out;
            out.header.stamp    = ros::Time::now();
            out.header.frame_id = "map";
            {
                std::lock_guard<std::mutex> lk(g_st.mtx);
                g_st.pos_x += dx_map;
                g_st.pos_y += dy_map;
                g_st.pos_z  = height_m;

                out.pose.position.x = g_st.pos_x;
                out.pose.position.y = g_st.pos_y;
                out.pose.position.z = g_st.pos_z;

                // 朝向复用最新 ToF 朝向（仅 yaw）
                const double hz = g_st.heading / 2.0;
                out.pose.orientation.x = 0.0;
                out.pose.orientation.y = 0.0;
                out.pose.orientation.z = std::sin(hz);
                out.pose.orientation.w = std::cos(hz);
            }
            pose_pub.publish(out);

            path_msg.header.stamp = out.header.stamp;
            path_msg.poses.push_back(out);
            path_pub.publish(path_msg);

            ROS_INFO_THROTTLE(2.0,
                "光流里程计: pos=(%.3f, %.3f, %.3f)m  flow=[%d,%d] dt=%uµs h=%.3fm conf=%u",
                out.pose.position.x, out.pose.position.y, out.pose.position.z,
                flow_x, flow_y, dt_us, height_m, lconf);
        }
    });

    ROS_INFO("oflow_odometry 启动: port=%s mount_yaw=%.1f° init_heading=%.1f°",
             port.c_str(), mount_yaw_deg, init_heading_deg);
    ros::spin();

    g_running = false;
    close(fd);
    serial_thread.join();
    return 0;
}
