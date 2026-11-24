"""Compara giro por /odom vs giroscopio (/imu) y saca desvio lateral de /odom en cada bag."""
import math, sys, pathlib
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
TS = get_typestore(Stores.ROS2_HUMBLE)

def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

root = pathlib.Path(sys.argv[1])
for bag in sorted(root.iterdir()):
    if not bag.is_dir():
        continue
    cmd, odom, imu = [], [], []
    with AnyReader([bag], default_typestore=TS) as r:
        for con, t, raw in r.messages():
            m = r.deserialize(raw, con.msgtype)
            ts = t * 1e-9
            if con.topic == '/cmd_vel':
                cmd.append((ts, m.linear.x, m.angular.z))
            elif con.topic == '/odom':
                p = m.pose.pose
                odom.append((ts, p.position.x, p.position.y, yaw(p.orientation)))
            elif con.topic == '/imu':
                imu.append((ts, m.angular_velocity.z, yaw(m.orientation)))
    cmd, odom, imu = map(np.array, (cmd, odom, imu))
    moving = (np.abs(cmd[:, 1]) > 1e-4) | (np.abs(cmd[:, 2]) > 1e-4)
    t0, t1 = cmd[moving, 0].min(), cmd[moving, 0].max()
    a, b = t0 - 0.3, t1 + 2.0
    # sesgo del giroscopio con el robot quieto antes de moverse
    quiet = imu[(imu[:, 0] < t0 - 0.2) & (imu[:, 0] > t0 - 5)]
    bias = quiet[:, 1].mean() if len(quiet) > 10 else 0.0
    w = imu[(imu[:, 0] >= a) & (imu[:, 0] <= b)]
    gyro = np.trapezoid(w[:, 1] - bias, w[:, 0])
    qyaw = math.atan2(math.sin(w[-1, 2] - w[0, 2]), math.cos(w[-1, 2] - w[0, 2]))
    o = odom[(odom[:, 0] >= a) & (odom[:, 0] <= b)]
    dyaw = np.unwrap(o[:, 3])
    dx, dy = o[-1, 1] - o[0, 1], o[-1, 2] - o[0, 2]
    th0 = o[0, 3]
    s = dx*math.cos(th0) + dy*math.sin(th0)
    d = -dx*math.sin(th0) + dy*math.cos(th0)
    print(f"{bag.name:10s} mov {t1-t0:5.1f}s  odom_yaw {math.degrees(dyaw[-1]-dyaw[0]):7.2f}  "
          f"gyro {math.degrees(gyro):7.2f}  imu_quat {math.degrees(qyaw):7.2f}  sesgo {math.degrees(bias):6.3f} deg/s  "
          f"avance {s*100:6.1f} cm  lateral {d*100:5.1f} cm  quiet_n {len(quiet)}")
