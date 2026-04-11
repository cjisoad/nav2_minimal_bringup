#!/usr/bin/env python3

import asyncio
import json
import time
from dataclasses import dataclass

import rclpy
import websockets
from geometry_msgs.msg import Twist
from rclpy.node import Node

WEBSOCKET_URL = "ws://192.168.4.1:8181"

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

class CarController:
    def __init__(self):
        self.ws = None
        self.running = False
        self.current_speeds = WheelSpeeds()
        self.base_speed = 50
        self.acceleration_time = 3
        self.forward_cmd = 0.0
        self.turn_cmd = 0.0
        self.last_cmd_time = 0.0
        self.cmd_timeout_sec = 0.5
        self.ros_node = None
        
    def calculate_wheel_speeds(self, forward: float, turn: float) -> WheelSpeeds:
        left_speed = (forward - turn * TURN_RATIO) * self.base_speed
        right_speed = (forward + turn * TURN_RATIO) * self.base_speed
        
        left_speed = max(-MAX_SPEED, min(MAX_SPEED, int(left_speed)))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, int(right_speed)))
        
        return WheelSpeeds(
            left_front=left_speed,
            right_front=right_speed,
            left_rear=left_speed,
            right_rear=right_speed
        )
    
    def create_motor_command(self, wheel_id: int, speed: int) -> dict:
        return {
            "T": 10010,
            "id": wheel_id,
            "cmd": speed,
            "act": self.acceleration_time
        }
    
    async def send_command(self, command: dict):
        if self.ws:
            try:
                await self.ws.send(json.dumps(command))
                print(f"Sent: {json.dumps(command)}")
            except Exception as e:
                print(f"Send error: {e}")
    
    async def send_wheel_speeds(self, speeds: WheelSpeeds):
        commands = [
            self.create_motor_command(WHEEL_IDS["left_front"], speeds.left_front),
            self.create_motor_command(WHEEL_IDS["right_front"], speeds.right_front),
            self.create_motor_command(WHEEL_IDS["left_rear"], speeds.left_rear),
            self.create_motor_command(WHEEL_IDS["right_rear"], speeds.right_rear)
        ]
        
        for cmd in commands:
            await self.send_command(cmd)
            await asyncio.sleep(0.01)
    
    async def set_heartbeat(self, time_ms: int = 1000):
        command = {"T": 11001, "time": time_ms}
        await self.send_command(command)
        print(f"Heartbeat set to {time_ms}ms")
    
    async def stop_all_wheels(self):
        await self.send_wheel_speeds(WheelSpeeds(0, 0, 0, 0))
        print("All wheels stopped")

    def update_cmd_vel(self, linear_x: float, angular_z: float):
        self.forward_cmd = max(-1.0, min(1.0, float(linear_x)))
        self.turn_cmd = max(-1.0, min(1.0, float(angular_z)))
        self.last_cmd_time = time.monotonic()

    async def control_loop(self):
        print("\n=== ROS2 /cmd_vel Control Active ===")
        print("Listening topic: /cmd_vel")
        print("Use geometry_msgs/msg/Twist commands")
        print("====================================\n")
        
        last_speeds = WheelSpeeds()
        
        while self.running and rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)

            if time.monotonic() - self.last_cmd_time > self.cmd_timeout_sec:
                forward = 0.0
                turn = 0.0
            else:
                forward = self.forward_cmd
                turn = self.turn_cmd
            
            new_speeds = self.calculate_wheel_speeds(forward, turn)
            
            if (new_speeds.left_front != last_speeds.left_front or
                new_speeds.right_front != last_speeds.right_front or
                new_speeds.left_rear != last_speeds.left_rear or
                new_speeds.right_rear != last_speeds.right_rear):
                
                await self.send_wheel_speeds(new_speeds)
                last_speeds = new_speeds
                print(f"Speeds - LF: {new_speeds.left_front:4d}, RF: {new_speeds.right_front:4d}, "
                      f"LR: {new_speeds.left_rear:4d}, RR: {new_speeds.right_rear:4d}")
            
            await asyncio.sleep(0.05)
        
        await self.stop_all_wheels()
    
    async def run(self):
        print(f"Connecting to {WEBSOCKET_URL}...")
        
        try:
            async with websockets.connect(WEBSOCKET_URL) as ws:
                self.ws = ws
                self.running = True
                self.ros_node = CmdVelSubscriber(self)
                print("Connected successfully!")
                
                await self.set_heartbeat(1000)
                
                await self.control_loop()
                
        except ConnectionRefusedError:
            print(f"Cannot connect to {WEBSOCKET_URL}")
            print("Please check:")
            print("  1. Is the car powered on?")
            print("  2. Are you connected to the car's WiFi?")
            print("  3. Is the IP address correct?")
        except Exception as e:
            print(f"Connection error: {e}")
        finally:
            self.running = False
            if self.ros_node is not None:
                self.ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            print("Controller stopped")


class CmdVelSubscriber(Node):
    def __init__(self, controller: CarController):
        super().__init__("car_controller")
        self.controller = controller
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.get_logger().info("Subscribed to /cmd_vel")

    def cmd_vel_callback(self, msg: Twist):
        self.controller.update_cmd_vel(msg.linear.x, msg.angular.z)

def main():
    print("=" * 50)
    print("  Four-Wheel Car ROS2 Controller")
    print("=" * 50)
    print()
    
    controller = CarController()
    
    try:
        rclpy.init()
        asyncio.run(controller.run())
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")

if __name__ == "__main__":
    main()
