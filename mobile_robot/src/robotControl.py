#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import sys
import select
import tty
import termios
import math
from statistics import mean
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('proportional_controller')

        qos_profile = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT, history = HistoryPolicy.KEEP_LAST, depth = 10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.imu_subscription = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback,qos_profile)
        self.joint_state_subscription = self.create_subscription(JointState,'joint_states', self.joint_state_callback, qos_profile)

        self.current_pos = [0,0]
        self.target_pos = [10,10]

        self.kp = 0.2
        self.control_output = 0.0
        self.yaw = 0.0

        self.time_period = 0.5
        self.timer = self.create_timer(self.time_period, self.publish_commands)

    def imu_callback(self, msg):
        quat_data = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w]
        (roll,pitch, self.yaw) = euler_from_quaternion(quat_data)
        # self.yaw -= 1.5707

    def joint_state_callback(self, msg):
        self.AvgAng_velocity= (msg.velocity[4]+msg.velocity[5])/2
        linear_velocity= 0.1015*self.AvgAng_velocity

        self.current_pos[0]+= linear_velocity*(math.cos(self.yaw))*self.time_period
        self.current_pos[1]+= linear_velocity*(math.sin(self.yaw))*self.time_period

        delta_x = self.target_pos[0] - self.current_pos[0]
        delta_y = self.target_pos[1] - self.current_pos[1]

        self.desired_yaw = np.arctan2(delta_y, delta_x)

        
    def publish_commands(self):
        error = self.desired_yaw - self.yaw
        print(self.yaw)
        Control_output = self.kp*error

        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()

        linear_vel = 3.0
        wheel_velocities.data = [0.0,0.0,linear_vel,-linear_vel]
        joint_positions.data = [Control_output,Control_output]

        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)

        
def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = IMUSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
