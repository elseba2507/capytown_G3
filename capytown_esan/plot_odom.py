import os, sys, math, yaml
import numpy as np
import matplotlib
matplotlib.use("Agg")  # el robot no tiene pantalla: guarda PNG sin abrir ventanas
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def storage_id(bag):
    # lee el formato del bag (sqlite3 o mcap) desde su metadata.yaml
    with open(os.path.join(bag, "metadata.yaml")) as f:
        return yaml.safe_load(f)["rosbag2_bagfile_information"]["storage_identifier"]


def read_pose(bag, topic="/odom"):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id=storage_id(bag)), ConverterOptions("", ""))
    t, x, y, th = [], [], [], []
    while r.has_next():
        tp, data, stamp = r.read_next()
        if tp != topic:
            continue
        m = deserialize_message(data, Odometry)
        q = m.pose.pose.orientation
        t.append(stamp * 1e-9)
        x.append(m.pose.pose.position.x)
        y.append(m.pose.pose.position.y)
        th.append(math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))
    return t, x, y, th


if __name__ == "__main__":
    bag = sys.argv[1].rstrip("/")
    name = os.path.basename(bag)
    t, x, y, th = read_pose(bag)

    # grafica 1: trayectoria x-y
    plt.figure()
    plt.plot(x, y)
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(name + ": trayectoria x-y")
    plt.savefig(name + "_xy.png", dpi=150)

    # grafica 2: orientacion theta(t); unwrap quita los saltos de +-180 grados
    plt.figure()
    plt.plot([ti - t[0] for ti in t], np.degrees(np.unwrap(th)))
    plt.xlabel("t [s]")
    plt.ylabel("theta [grados]")
    plt.title(name + ": orientacion theta(t)")
    plt.savefig(name + "_theta.png", dpi=150)
