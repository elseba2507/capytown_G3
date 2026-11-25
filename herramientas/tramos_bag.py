"""Por cada tramo recto (cmd_vel v > 0) y cada giro de un bag: cambio de rumbo segun /odom y segun el giroscopio."""
import math, sys, pathlib
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
TS = get_typestore(Stores.ROS2_HUMBLE)

def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

for bag in sys.argv[1:]:
    cmd, odom, imu = [], [], []
    with AnyReader([pathlib.Path(bag)], default_typestore=TS) as r:
        for con, t, raw in r.messages():
            m = r.deserialize(raw, con.msgtype); ts = t * 1e-9
            if con.topic == '/cmd_vel': cmd.append((ts, m.linear.x, m.angular.z))
            elif con.topic == '/odom': odom.append((ts, yaw(m.pose.pose.orientation)))
            elif con.topic == '/imu': imu.append((ts, m.angular_velocity.z))
    cmd, odom, imu = map(np.array, (cmd, odom, imu))
    oy = np.unwrap(odom[:, 1])
    # sesgo: primer segundo con cmd en cero al inicio
    quiet = imu[imu[:, 0] < cmd[(np.abs(cmd[:, 1]) + np.abs(cmd[:, 2])) > 1e-4, 0].min()]
    b = quiet[-30:, 1].mean() if len(quiet) else 0.0
    mode = np.where(cmd[:, 1] > 1e-4, 'R', np.where(np.abs(cmd[:, 2]) > 1e-4, 'G', '-'))
    print(f"== {pathlib.Path(bag).name}  sesgo inicial {math.degrees(b):.3f} deg/s")
    i = 0
    while i < len(mode):
        j = i
        while j + 1 < len(mode) and mode[j + 1] == mode[i]: j += 1
        t0, t1 = cmd[i, 0], cmd[j, 0]
        if mode[i] != '-' and t1 - t0 > 1.0:
            w = imu[(imu[:, 0] >= t0) & (imu[:, 0] <= t1)]
            g = np.trapezoid(w[:, 1] - b, w[:, 0])
            o = np.interp([t0, t1], odom[:, 0], oy)
            print(f"  {mode[i]} {t1-t0:5.1f}s  gyro {math.degrees(g):7.2f}  odom {math.degrees(o[1]-o[0]):7.2f}  odom-gyro {math.degrees(o[1]-o[0]-g):6.2f}")
        elif mode[i] == '-' and t1 - t0 > 5:
            w = imu[(imu[:, 0] >= t0) & (imu[:, 0] <= t1)]
            g = np.trapezoid(w[:, 1] - b, w[:, 0])
            print(f"  quieto {t1-t0:5.1f}s gyro {math.degrees(g):6.2f} (deriva {math.degrees(g)/(t1-t0):.3f} deg/s o giro a mano)")
        i = j + 1
