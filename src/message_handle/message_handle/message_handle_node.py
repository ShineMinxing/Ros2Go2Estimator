#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros

def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角 (roll, pitch, yaw) 转换为四元数 (x, y, z, w)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw

def euler_from_matrix(matrix):
    """
    实现等效于tf_transformations.euler_from_matrix(matrix, axes='sxyz')
    返回顺序为 (roll, pitch, yaw) 的XYZ欧拉角
    """
    epsilon = 1e-6
    R = np.array(matrix, dtype=np.float64)
    
    # 提取各轴旋转角度
    if abs(R[2,0]) < 1.0 - epsilon:
        pitch = math.asin(-R[2,0])
        roll = math.atan2(R[2,1], R[2,2])
        yaw = math.atan2(R[1,0], R[0,0])
    else:
        # 奇异情况处理
        sign = math.copysign(1.0, R[2,0])
        pitch = -sign * math.pi/2
        roll = -sign * math.atan2(R[0,1], R[0,2])
        yaw = 0.0
    
    return roll, pitch, yaw

def euler_to_matrix(roll, pitch, yaw):
    """创建仅包含指定欧拉角的旋转矩阵（XYZ顺序）"""
    # 绕X轴旋转
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])
    
    # 绕Y轴旋转
    Ry = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])
    
    # 绕Z轴旋转
    Rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # 组合旋转矩阵（XYZ顺序）
    return Rz @ Ry @ Rx

class MssageHandleNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan_custom')

        # TF组件初始化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster1 = tf2_ros.TransformBroadcaster(self)
        self.tf_broadcaster2 = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.pointcloud_callback,
            10
        )
        self.publisher_ = self.create_publisher(LaserScan, '/SMXFE/Scan', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/SMXFE/Odom',
            self.odom_callback,
            10
        )

        self.odom_2D_sub = self.create_subscription(
            Odometry,
            '/SMXFE/Odom_2D',
            self.odom_2D_callback,
            10
        )

        # LaserScan 参数配置
        self.angle_min = -math.pi / 2
        self.angle_max = math.pi / 2
        self.angle_increment = 0.0056
        self.range_min = 0.1
        self.range_max = 30.0
        self.min_height = 0.5
        self.max_height = 1.0
        self.odom_counter = 0
        self.odom_2D_counter = 0

        self.publish_base_to_utlidar_transform()  # 原有base到lidar的发布
        self.publish_map_to_odom_transform()      # 新增map到odom的发布

        self.get_logger().info("MssageHandleNode 已启动")

    def publish_base_to_utlidar_transform(self):
        """
        发布 base_link 到 utlidar_lidar 的静态TF变换，
        参考URDF中 radar_joint 的origin: translation (0.28945, 0, -0.046825) 和 rpy (0, 2.8782, 0)
        """
        static_transform_BU = TransformStamped()
        static_transform_BU.header.stamp = self.get_clock().now().to_msg()
        static_transform_BU.header.frame_id = "base_link"         # 父坐标系
        static_transform_BU.child_frame_id = "utlidar_lidar"        # 子坐标系

        # 设置平移
        static_transform_BU.transform.translation.x = 0.28945
        static_transform_BU.transform.translation.y = 0.0
        static_transform_BU.transform.translation.z = -0.046825

        # 设置旋转：将 rpy 转换为四元数
        qx, qy, qz, qw = euler_to_quaternion(0.0, 2.8782, 0.0)
        static_transform_BU.transform.rotation.x = qx
        static_transform_BU.transform.rotation.y = qy
        static_transform_BU.transform.rotation.z = qz
        static_transform_BU.transform.rotation.w = qw

        self.static_broadcaster.sendTransform([static_transform_BU])
        self.get_logger().info("Published static transform from base_link to utlidar_lidar")

    def publish_map_to_odom_transform(self):
        """
        发布 map 到 odom 的静态TF变换（无平移和旋转）
        """
        static_transform_mo = TransformStamped()
        static_transform_mo.header.stamp = self.get_clock().now().to_msg()
        static_transform_mo.header.frame_id = "map"    # 父坐标系
        static_transform_mo.child_frame_id = "odom"   # 子坐标系

        # 设置平移：无偏移
        static_transform_mo.transform.translation.x = 0.0
        static_transform_mo.transform.translation.y = 0.0
        static_transform_mo.transform.translation.z = 0.0

        # 设置旋转：无旋转（单位四元数）
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, 0.0)  # rpy均为0
        static_transform_mo.transform.rotation.x = qx
        static_transform_mo.transform.rotation.y = qy
        static_transform_mo.transform.rotation.z = qz
        static_transform_mo.transform.rotation.w = qw

        self.static_broadcaster.sendTransform([static_transform_mo])
        self.get_logger().info("Published static transform from map to odom")


    def transform_to_matrix(self, transform: TransformStamped) -> np.ndarray:
        """
        将 TransformStamped 转换为 4x4 齐次变换矩阵
        """
        t = transform.transform.translation
        r = transform.transform.rotation
        x, y, z, w = r.x, r.y, r.z, r.w
        # 使用四元数转换公式构造旋转矩阵
        R = np.array([
            [1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,       2*x*z + 2*y*w],
            [2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z,   2*y*z - 2*x*w],
            [2*x*z - 2*y*w,       2*y*z + 2*x*w,       1 - 2*x*x - 2*y*y]
        ])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.array([t.x, t.y, t.z])
        return T

    def pointcloud_callback(self, msg):
        try:
            # 查找从 msg.header.frame_id 到 base_link 的变换
            transform_to_baselink = self.tf_buffer.lookup_transform(
                'base_link_2D',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f'Transform error: {str(e)}')
            return

        try:
             # 2) 分别转成 4x4 矩阵
            T = self.transform_to_matrix(transform_to_baselink)     # msg.header.frame_id -> base_link

            # 使用列表推导式生成每个点的浮点数列表，确保数据类型一致
            points_list = [list(p) for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
            if not points_list:
                self.get_logger().warn("No points found in point cloud")
                return

            points = np.array(points_list, dtype=np.float64)
            # 如果只有一个点，则 reshape 为二维数组
            if points.ndim == 1:
                points = points.reshape(1, -1)

            num_points = points.shape[0]
            ones = np.ones((num_points, 1), dtype=np.float64)
            points_hom = np.hstack((points, ones))
            transformed_points_hom = (T @ points_hom.T).T
            transformed_points = transformed_points_hom[:, :3]

            fields = [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1)
            ]
            cloud_in_base = pc2.create_cloud(msg.header, fields, transformed_points.tolist())
        except Exception as e:
            self.get_logger().error(f'Cloud transform_to_baselink failed: {str(e)}')
            return

        # 构建 LaserScan 消息
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link_2D'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        num_bins = int((self.angle_max - self.angle_min) / self.angle_increment)+2
        ranges = [float('inf')] * num_bins

        for point in pc2.read_points(cloud_in_base, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            if z < self.min_height or z > self.max_height:
                continue
            angle = math.atan2(y, x)
            if angle < self.angle_min or angle > self.angle_max:
                continue
            r = math.sqrt(x**2 + y**2)
            index = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= index < num_bins and r < ranges[index]:
                ranges[index] = r

        scan.ranges = ranges
        self.publisher_.publish(scan)
        # self.get_logger().info('Published scan with %d points' % len(ranges))

    def odom_callback(self, msg):
        self.odom_counter += 1
        
        # 增加计数器，每50条消息发布一次 base_link 到 utlidar_lidar 的TF
        if self.odom_counter % 5 == 0:
            self.odom_counter = 0
            """处理里程计数据并发布 odom 到 base_link 的TF变换"""
            try:
                # 发布 odom 到 base_link 的TF
                transform = TransformStamped()
                transform.header.stamp = msg.header.stamp
                transform.header.frame_id = "odom"       # 父坐标系
                transform.child_frame_id = "base_link"     # 子坐标系
                transform.transform.translation.x = msg.pose.pose.position.x
                transform.transform.translation.y = msg.pose.pose.position.y
                transform.transform.translation.z = msg.pose.pose.position.z
                transform.transform.rotation = msg.pose.pose.orientation
                self.tf_broadcaster1.sendTransform(transform)
            except Exception as e:
                self.get_logger().error(f"odom-baseTF发布失败: {str(e)}")

            try:
                static_transform = TransformStamped()
                static_transform.header.stamp = msg.header.stamp
                static_transform.header.frame_id = "base_link"        # 父坐标系
                static_transform.child_frame_id = "utlidar_lidar"       # 子坐标系
                # 设置平移
                static_transform.transform.translation.x = 0.28945
                static_transform.transform.translation.y = 0.0
                static_transform.transform.translation.z = -0.046825
                # 将 rpy 转换为四元数
                qx, qy, qz, qw = euler_to_quaternion(0.0, 2.8782, 0.0)
                static_transform.transform.rotation.x = qx
                static_transform.transform.rotation.y = qy
                static_transform.transform.rotation.z = qz
                static_transform.transform.rotation.w = qw

                self.tf_broadcaster1.sendTransform(static_transform)
            except Exception as e:
                self.get_logger().error(f"base-lidar TF发布失败: {str(e)}")

    def odom_2D_callback(self, msg):
        self.odom_2D_counter += 1
        
        # 增加计数器，每50条消息发布一次 base_link 到 utlidar_lidar 的TF
        if self.odom_2D_counter % 5 == 0:
            self.odom_2D_counter = 0
            """处理里程计数据并发布 odom 到 base_link 的TF变换"""
            try:
                # 发布 odom 到 base_link 的TF
                transform = TransformStamped()
                transform.header.stamp = msg.header.stamp
                transform.header.frame_id = "odom"       # 父坐标系
                transform.child_frame_id = "base_link_2D"     # 子坐标系
                transform.transform.translation.x = msg.pose.pose.position.x
                transform.transform.translation.y = msg.pose.pose.position.y
                transform.transform.translation.z = msg.pose.pose.position.z
                transform.transform.rotation = msg.pose.pose.orientation
                self.tf_broadcaster2.sendTransform(transform)
            except Exception as e:
                self.get_logger().error(f"odom-baseTF发布失败: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MssageHandleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
