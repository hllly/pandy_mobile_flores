#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class PathTrackingNode(Node):
    def __init__(self):
        super().__init__('path_tracking_node')

        self.tf_broadcaster = TransformBroadcaster(self)
        # 订阅机器人当前位姿
        self.pose_sub = self.create_subscription(
            Odometry,
            'HighState',
            self.pose_callback,
            1)
            
        # 发布速度指令
        self.cmd_pub = self.create_publisher(
            Odometry,
            'HighCmd',
            1)
        
        self.path_pub = self.create_publisher(
            MarkerArray, 
            'figure8_markers', 
            10)
            
        # 初始化PID参数
        # self.kp = 3
        # self.ki = 0.0001
        # self.kd = 0.5
        self.kp = 1
        self.ki = 0.0001
        self.kd = 0.1
        
        # PID误差
        self.last_error = 0.0
        self.integral = 0.0
        
        # 存储当前位姿
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # 定义目标路径点
        # self.path_points = [
        #     [-1.0, 2.0],
        #     [-2.0, -2.0],
        #     [-3.0, -1.0]
        # ]
        # self.path_points = self.generate_figure8_points(num_points=100, scale_x=8.0, scale_y=5.0)
        # self.target_points = [
        #     [0,0],
        #     [1.9,0],
        #     [1.9,-2.5],
        #     [4.4,-2.5],
        #     [4.4,-4.4],
        #     [1.9,-4.4],
        #     [1.9,-6.8],
        #     [0,-6.8],
        #     [0,0],
        # ]
        self.target_points = [
            [0,0],
            [0,-5]
        ]
        self.path_points = self.interpolate_path(self.target_points, num_interpolation_points=100)
        # self.path_points[:,0] -= 10
        self.publish_path(self.path_points, r=1.0, g=0.0, b=0.0)

        self.current_target_index = 0
        
        # 控制频率10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def pose_callback(self, msg):
        # 更新当前位姿
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 从四元数转换为欧拉角
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = self.euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w)
        
        # 创建TransformStamped消息
        t = TransformStamped()
        
        # 设置时间戳
        t.header.stamp = self.get_clock().now().to_msg()
        
        # 设置坐标系
        t.header.frame_id = 'world'     # 父坐标系
        t.child_frame_id = 'base'  # 子坐标系
        
        # 设置位置（从你的消息中获取机器人位置）
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # 设置旋转（四元数）
        t.transform.rotation.w = msg.pose.pose.orientation.w
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y    
        t.transform.rotation.z = msg.pose.pose.orientation.z
        
        # 发布transform
        self.tf_broadcaster.sendTransform(t)
    
    def euler_from_quaternion(self, x, y, z, w):
        # 四元数转欧拉角
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw
        
    def get_target_point(self):
        # 获取当前目标点
        if self.current_target_index >= len(self.path_points):
            return None
        return self.path_points[self.current_target_index]
    
    def distance_to_target(self, target_point):
        # 计算到目标点距离
        dx = target_point[0] - self.current_x
        dy = target_point[1] - self.current_y
        return math.sqrt(dx*dx + dy*dy)
    
    def angle_to_target(self, target_point):
        # 计算目标点角度
        dx = target_point[0] - self.current_x
        dy = target_point[1] - self.current_y
        target_angle = math.atan2(dy, dx)
        
        # 计算角度差
        angle_diff = target_angle - self.current_yaw + math.pi/2
        # 归一化到[-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2*math.pi
        while angle_diff < -math.pi:
            angle_diff += 2*math.pi
            
        return angle_diff
    
    def interpolate_path(self, path_points, num_interpolation_points):
        interpolated_points = []
        
        # 遍历所有相邻的点对
        for i in range(len(path_points)-1):
            # 获取当前点和下一个点
            current_point = path_points[i]
            next_point = path_points[i+1]
            
            # 添加当前点
            interpolated_points.append(current_point)
            
            # 计算两点之间的插值点
            for j in range(num_interpolation_points):
                # 计算插值比例
                t = (j + 1) / (num_interpolation_points + 1)
                
                # 线性插值
                interpolated_x = current_point[0] + t * (next_point[0] - current_point[0])
                interpolated_y = current_point[1] + t * (next_point[1] - current_point[1])
                
                interpolated_points.append(np.array([interpolated_x, interpolated_y]))
        
        # 添加最后一个点
        interpolated_points.append(path_points[-1])
        
        return np.array(interpolated_points)
    
    def control_loop(self):
        target_point = self.get_target_point()
        if target_point is None:
            # 路径完成
            self.stop_robot()
            return
            
        # 检查是否到达当前目标点
        if self.distance_to_target(target_point) < 0.4:  # 阈值0.1米
            self.current_target_index += 1
            if self.current_target_index < len(self.path_points):
                print("current target index is", self.current_target_index)
                print("current target point is", self.path_points[self.current_target_index])
            return
            
        # 计算角度误差
        angle_error = self.angle_to_target(target_point)
        
        # PID控制
        self.integral += angle_error * 0.1  # dt = 0.1s
        derivative = (angle_error - self.last_error) / 0.1
        
        # 计算角速度
        angular_z = self.kp * angle_error + \
                   self.ki * self.integral + \
                   self.kd * derivative
                   
        self.last_error = angle_error

        if angular_z > 2.0:
            angular_z = 2.0
        elif angular_z < -2.0:
            angular_z = -2.0
        
        # 创建并发布速度指令
        cmd = Odometry()
        cmd.twist.twist.linear.x = 0.0  # 固定线速度
        cmd.twist.twist.linear.y = 0.5  # 固定线速度
        cmd.twist.twist.angular.z = -angular_z
        cmd.pose.pose.position.x = self.path_points[self.current_target_index][0]
        cmd.pose.pose.position.x = self.path_points[self.current_target_index][1]
        self.cmd_pub.publish(cmd)
        
    def stop_robot(self):
        cmd = Odometry()
        cmd.twist.twist.linear.x = 0.0
        cmd.twist.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def generate_figure8_points(self,num_points=100, scale_x=1.0, scale_y=1.0):
        # 参数t
        t = np.linspace(0, 2*np.pi, num_points)
        
        # 生成8字形坐标
        x = scale_x * np.sin(t)
        y = scale_y * np.sin(t) * np.cos(t)
        
        # 组合坐标
        points = np.column_stack((x, y))
        return points
    
    def publish_path(self, points, r=1.0, g=0.0, b=0.0):
        marker_array = MarkerArray()
        
        # 创建线条marker
        point_marker = Marker()
        point_marker.header.frame_id = "world"
        point_marker.header.stamp = self.get_clock().now().to_msg()
        point_marker.type = Marker.POINTS
        point_marker.action = Marker.ADD
        point_marker.scale.x = 0.1  # 线宽
        point_marker.scale.y = 0.1 
        point_marker.color.r = r
        point_marker.color.g = g
        point_marker.color.b = b
        point_marker.color.a = 1.0
        point_marker.id = 0
        
        # 添加点
        for point in points:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = float(0.5)
            point_marker.points.append(p)
        
        marker_array.markers.append(point_marker)
        self.path_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()