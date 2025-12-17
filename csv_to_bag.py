#!/usr/bin/env python3
import csv
import os
import shutil

import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Imu


CSV_PATH = "output_20250826_234506.csv"
BAG_URI = "stair_test"

# 固定频率（Hz）
SAMPLE_RATE = 400.0


def detect_delimiter(path: str) -> str:
    """简单检测一下是逗号还是制表符"""
    with open(path, "r", newline="") as f:
        first_line = f.readline()
    if first_line.count("\t") > first_line.count(","):
        return "\t"
    else:
        return ","


def main():
    if not os.path.exists(CSV_PATH):
        raise FileNotFoundError(f"找不到 CSV 文件: {CSV_PATH}")

    # 如果 ground_test 已经存在，先删掉（覆盖）
    if os.path.exists(BAG_URI):
        print(f"[INFO] 删除已有 bag 目录: {BAG_URI}/")
        shutil.rmtree(BAG_URI)

    delimiter = detect_delimiter(CSV_PATH)
    print(f"[INFO] 使用分隔符: {repr(delimiter)}")

    # 初始化 rosbag2 写入器
    storage_options = rosbag2_py.StorageOptions(
        uri=BAG_URI,
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)

    # 注册两个 topic
    joint_topic = rosbag2_py.TopicMetadata(
        name="SMX/Go2Joint",
        type="std_msgs/msg/Float64MultiArray",
        serialization_format="cdr",
    )
    imu_topic = rosbag2_py.TopicMetadata(
        name="SMX/Go2IMU",
        type="sensor_msgs/msg/Imu",
        serialization_format="cdr",
    )
    writer.create_topic(joint_topic)
    writer.create_topic(imu_topic)

    # 需要的关节索引（共 12 个）
    desired_joints = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]

    with open(CSV_PATH, "r", newline="") as f:
        reader = csv.DictReader(f, delimiter=delimiter)

        # 规范化列名（去掉 BOM 和前后空格）
        raw_fieldnames = reader.fieldnames
        if raw_fieldnames is None:
            raise RuntimeError("CSV 文件没有读到表头（fieldnames is None）")

        norm_fieldnames = [fn.strip().lstrip("\ufeff") for fn in raw_fieldnames]
        name_map = dict(zip(raw_fieldnames, norm_fieldnames))

        print("[INFO] 原始列名示例：", raw_fieldnames[:10])
        print("[INFO] 规范列名示例：", norm_fieldnames[:10])

        for row_idx, row in enumerate(reader):
            # 用行号构造理想 400Hz 时间戳
            t_rel = row_idx / SAMPLE_RATE   # 秒
            stamp_ns = int(t_rel * 1e9)

            # 把这一行的 key 规范化一下
            nrow = {name_map[k]: v for k, v in row.items()}

            # 2) 关节数据 -> Float64MultiArray
            q = []
            dq = []
            tor = []
            for j in desired_joints:
                q.append(float(nrow[f"posJoint{j}"]))
                dq.append(float(nrow[f"velJoint{j}"]))
                tor.append(float(nrow[f"torJoint{j}"]))

            data = q + dq + tor  # 共 36 个元素

            joint_msg = Float64MultiArray()
            dim = MultiArrayDimension()
            dim.label = "joint_data"
            dim.size = len(data)
            dim.stride = len(data)

            layout = MultiArrayLayout()
            layout.dim = [dim]
            layout.data_offset = 0

            joint_msg.layout = layout
            joint_msg.data = data

            writer.write(
                "SMX/Go2Joint",
                serialize_message(joint_msg),
                stamp_ns,
            )

            # 3) IMU 数据 -> sensor_msgs/Imu
            imu_msg = Imu()
            # header 时间，同样用 t_rel
            sec = int(t_rel)
            nsec = int((t_rel - sec) * 1e9)
            imu_msg.header.stamp.sec = sec
            imu_msg.header.stamp.nanosec = nsec
            imu_msg.header.frame_id = "go2_imu"

            imu_msg.linear_acceleration.x = float(nrow["imu_acc_x"])
            imu_msg.linear_acceleration.y = float(nrow["imu_acc_y"])
            imu_msg.linear_acceleration.z = float(nrow["imu_acc_z"])

            imu_msg.angular_velocity.x = float(nrow["imu_gyro_x"])
            imu_msg.angular_velocity.y = float(nrow["imu_gyro_y"])
            imu_msg.angular_velocity.z = float(nrow["imu_gyro_z"])

            imu_msg.orientation.w = float(nrow["imu_quand_w"])
            imu_msg.orientation.x = float(nrow["imu_quand_x"])
            imu_msg.orientation.y = float(nrow["imu_quand_y"])
            imu_msg.orientation.z = float(nrow["imu_quand_z"])

            writer.write(
                "SMX/Go2IMU",
                serialize_message(imu_msg),
                stamp_ns,
            )

            if row_idx % 1000 == 0:
                print(f"写入第 {row_idx} 行, t = {t_rel:.3f} s")

    print(f"完成写入 bag: {BAG_URI}/")


if __name__ == "__main__":
    main()
