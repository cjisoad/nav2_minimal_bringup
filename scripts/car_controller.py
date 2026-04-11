#!/usr/bin/env python3
"""
ROS车辆导航控制器

功能：接收ROS导航系统的Twist指令，结合里程计数据，实现车辆到达目标点的控制
作者：ROS Navigation Controller
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import Imu
import math
import json
import serial
import serial.tools.list_ports
import asyncio
import threading
from dataclasses import dataclass
from typing import Optional, Tuple, List
import time


SERIAL_PORT = "/dev/tty.usbmodem5B5E0684011"
BAUD_RATE = 115200

WHEEL_IDS = {
    "left_front": 1,
    "right_front": 2,
    "left_rear": 3,
    "right_rear": 4
}

MAX_SPEED = 100
TURN_RATIO = 0.5


@dataclass
class WheelSpeeds:
    left_front: int = 0
    right_front: int = 0
    left_rear: int = 0
    right_rear: int = 0


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class Goal:
    x: float
    y: float
    theta: Optional[float] = None
    tolerance_xy: float = 0.1
    tolerance_theta: float = 0.1


class CarController(Node):
    """
    ROS车辆导航控制器节点
    
    功能：
    - 订阅里程计信息（/odom）
    - 订阅导航指令（/cmd_vel）
    - 发布车辆控制指令（/cmd_vel_out）
    - 目标点管理
    - 运动控制算法实现
    """
    
    def __init__(self):
        super().__init__('car_controller')
        
        self.get_logger().info('车辆导航控制器初始化...')
        
        self.serial = None
        self.serial_lock = threading.Lock()
        
        self.current_pose = Pose2D()
        self.target_pose = Pose2D()
        self.current_velocity = Twist()
        self.target_goal: Optional[Goal] = None
        
        self.base_speed = 50
        self.acceleration_time = 3
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', SERIAL_PORT),
                ('baud_rate', BAUD_RATE),
                ('max_speed', MAX_SPEED),
                ('turn_ratio', TURN_RATIO),
                ('control_frequency', 20.0),
                ('goal_tolerance_xy', 0.1),
                ('goal_tolerance_theta', 0.1),
                ('min_angular_velocity', 0.1),
                ('max_angular_velocity', 2.0),
                ('base_angular_velocity', 0.5),
            ]
        )
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.max_speed = self.get_parameter('max_speed').value
        self.turn_ratio = self.get_parameter('turn_ratio').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.goal_tolerance_xy = self.get_parameter('goal_tolerance_xy').value
        self.goal_tolerance_theta = self.get_parameter('goal_tolerance_theta').value
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_out',
            qos_profile
        )
        
        self.init_serial_connection()
        
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop
        )
        
        self.get_logger().info('车辆导航控制器已启动')
        self.get_logger().info(f'串口配置: {self.serial_port} @ {self.baud_rate} baud')
    
    def init_serial_connection(self):
        """初始化串口连接"""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f'成功连接到 {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'无法连接到串口: {e}')
            self.list_available_ports()
            self.serial = None
    
    def list_available_ports(self):
        """列出所有可用的串口"""
        ports = serial.tools.list_ports.comports()
        self.get_logger().info('可用串口列表:')
        for port in ports:
            self.get_logger().info(f'  - {port.device}: {port.description}')
    
    def odom_callback(self, msg: Odometry):
        """
        里程计回调函数
        
        解析里程计消息，提取当前位置和姿态
        Odometry消息结构：
        - header: 时间戳和坐标系
        - child_frame_id: 速度信息的坐标系
        - pose: 位置和姿态估计（包含协方差）
        - twist: 线速度和角速度（包含协方差）
        """
        try:
            self.current_pose.x = msg.pose.pose.position.x
            self.current_pose.y = msg.pose.pose.position.y
            self.current_pose.theta = self.quaternion_to_yaw(
                msg.pose.pose.orientation
            )
            
            self.current_velocity = msg.twist.twist
            
            self.get_logger().debug(
                f'位置: ({self.current_pose.x:.3f}, {self.current_pose.y:.3f}), '
                f'朝向: {math.degrees(self.current_pose.theta):.1f}°'
            )
        except Exception as e:
            self.get_logger().error(f'里程计数据解析错误: {e}')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Twist指令回调函数
        
        接收ROS导航系统的控制指令
        Twist消息结构：
        - linear: Vector3 (x, y, z) - 线速度分量
        - angular: Vector3 (x, y, z) - 角速度分量
        
        主要使用：
        - linear.x: 前向线速度 (m/s)
        - angular.z: 转向角速度 (rad/s)
        """
        try:
            target_linear = msg.linear.x
            target_angular = msg.angular.z
            
            self.get_logger().debug(
                f'收到Twist指令: linear.x={target_linear:.3f}m/s, '
                f'angular.z={target_angular:.3f}rad/s'
            )
            
            wheel_speeds = self.calculate_wheel_speeds(
                target_linear,
                target_angular
            )
            
            self.send_wheel_speeds_async(wheel_speeds)
            
            twist_out = Twist()
            twist_out.linear.x = target_linear
            twist_out.angular.z = target_angular
            self.cmd_vel_publisher.publish(twist_out)
            
        except Exception as e:
            self.get_logger().error(f'Twist指令处理错误: {e}')
    
    def goal_callback(self, msg: PoseStamped):
        """
        目标点回调函数
        
        接收导航系统的目标点设置
        """
        try:
            self.target_goal = Goal(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                theta=self.quaternion_to_yaw(msg.pose.orientation),
                tolerance_xy=self.goal_tolerance_xy,
                tolerance_theta=self.goal_tolerance_theta
            )
            
            self.get_logger().info(
                f'收到目标点: ({self.target_goal.x:.3f}, {self.target_goal.y:.3f}), '
                f'朝向: {math.degrees(self.target_goal.theta) if self.target_goal.theta else 0:.1f}°'
            )
        except Exception as e:
            self.get_logger().error(f'目标点解析错误: {e}')
    
    def control_loop(self):
        """
        主控制循环
        
        根据当前状态和目标点，计算并发送控制指令
        """
        if self.target_goal is None:
            return
        
        dx = self.target_goal.x - self.current_pose.x
        dy = self.target_goal.y - self.current_pose.y
        distance_to_goal = math.sqrt(dx * dx + dy * dy)
        
        if distance_to_goal < self.target_goal.tolerance_xy:
            self.get_logger().info('已到达目标点!')
            self.stop_vehicle()
            self.target_goal = None
            return
        
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_pose.theta)
        
        Kp_angular = 2.0
        angular_vel = Kp_angular * angle_diff
        angular_vel = max(-2.0, min(2.0, angular_vel))
        
        Kp_linear = 1.0
        linear_vel = Kp_linear * distance_to_goal
        linear_vel = max(0.0, min(0.5, linear_vel))
        
        if abs(angle_diff) > math.pi / 6:
            linear_vel *= 0.3
        
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        
        self.cmd_vel_publisher.publish(cmd_vel)
        
        self.get_logger().debug(
            f'控制: 距离={distance_to_goal:.3f}m, '
            f'角度差={math.degrees(angle_diff):.1f}°, '
            f'线速度={linear_vel:.3f}m/s, '
            f'角速度={angular_vel:.3f}rad/s'
        )
    
    def calculate_wheel_speeds(self, forward: float, turn: float) -> WheelSpeeds:
        """
        计算各轮速度
        
        Args:
            forward: 前进方向 (1.0=全速前进, -1.0=全速后退, 0.0=停止)
            turn: 转向 (-1.0=左转, 1.0=右转, 0.0=直行)
        
        Returns:
            WheelSpeeds: 各轮速度
        """
        left_speed = (forward + turn * self.turn_ratio) * self.base_speed
        right_speed = (forward - turn * self.turn_ratio) * self.base_speed
        
        left_speed = max(-self.max_speed, min(self.max_speed, int(left_speed)))
        right_speed = max(-self.max_speed, min(self.max_speed, int(right_speed)))
        
        return WheelSpeeds(
            left_front=left_speed,
            right_front=right_speed,
            left_rear=left_speed,
            right_rear=right_speed
        )
    
    def create_motor_command(self, wheel_id: int, speed: int) -> dict:
        """创建电机控制命令"""
        return {
            "T": 10010,
            "id": wheel_id,
            "cmd": speed,
            "act": self.acceleration_time
        }
    
    def send_wheel_speeds_async(self, speeds: WheelSpeeds):
        """异步发送轮速指令"""
        def send_task():
            with self.serial_lock:
                if self.serial and self.serial.is_open:
                    try:
                        commands = [
                            self.create_motor_command(WHEEL_IDS["left_front"], speeds.left_front),
                            self.create_motor_command(WHEEL_IDS["right_front"], -speeds.right_front),
                            self.create_motor_command(WHEEL_IDS["left_rear"], speeds.left_rear),
                            self.create_motor_command(WHEEL_IDS["right_rear"], -speeds.right_rear)
                        ]
                        
                        for cmd in commands:
                            message = json.dumps(cmd) + '\n'
                            self.serial.write(message.encode('utf-8'))
                            self.serial.flush()
                            time.sleep(0.01)
                        
                        self.get_logger().debug(
                            f'轮速: 左前={speeds.left_front}, 右前={speeds.right_front}, '
                            f'左后={speeds.left_rear}, 右后={speeds.right_rear}'
                        )
                    except Exception as e:
                        self.get_logger().error(f'串口发送错误: {e}')
        
        thread = threading.Thread(target=send_task, daemon=True)
        thread.start()
    
    def stop_vehicle(self):
        """停止车辆"""
        wheel_speeds = WheelSpeeds(0, 0, 0, 0)
        self.send_wheel_speeds_async(wheel_speeds)
        
        cmd_vel = Twist()
        self.cmd_vel_publisher.publish(cmd_vel)
        
        self.get_logger().info('车辆已停止')
    
    @staticmethod
    def quaternion_to_yaw(quaternion: Quaternion) -> float:
        """从四元数提取偏航角"""
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def destroy_node(self):
        """清理节点资源"""
        self.get_logger().info('关闭车辆控制器...')
        self.stop_vehicle()
        
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
                self.get_logger().info('串口连接已关闭')
            except Exception as e:
                self.get_logger().error(f'关闭串口时出错: {e}')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    controller = CarController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('程序被用户中断')
    except Exception as e:
        print(f'节点运行错误: {e}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
