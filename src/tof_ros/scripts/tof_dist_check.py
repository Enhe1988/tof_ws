#!/usr/bin/env python3
"""
实时打印 TMF8829 每帧的距离统计，用于验证测距精度。
用法：
  1. 先启动 roslaunch tof_ros tof_receiver.launch
  2. 再运行本脚本：python3 tof_dist_check.py
  3. 把传感器放在离墙已知距离处（如 1.0m），观察输出的 mean 是否匹配
"""
import rospy
import numpy as np
from sensor_msgs.msg import Image

def cb(msg):
    # 32FC1 深度图，NaN = 无效
    raw = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    valid = raw[np.isfinite(raw)]
    n_total = raw.size
    n_valid = valid.size

    if n_valid == 0:
        rospy.logwarn_throttle(2.0, f"0/{n_total} 个有效测量，全部无效")
        return

    rospy.loginfo(
        f"有效={n_valid:4d}/{n_total}  "
        f"min={valid.min():.3f}m  "
        f"mean={valid.mean():.3f}m  "
        f"median={np.median(valid):.3f}m  "
        f"max={valid.max():.3f}m  "
        f"std={valid.std():.3f}m"
    )

rospy.init_node('tof_dist_check')
rospy.Subscriber('/tof/depth', Image, cb)
rospy.loginfo("等待 /tof/depth ... 按 Ctrl-C 退出")
rospy.spin()
