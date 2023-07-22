#!/usr/bin/env python3

# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time


class TestROS2Bridge(Node):
    def __init__(self):

        super().__init__("test_ros2bridge")

        # Create the publisher. This publisher will publish a JointState message to the /joint_command topic.
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Create a JointState message
        self.joint_state = JointState()

        self.joint_state.name = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "panda_finger_joint1",
            "panda_finger_joint2",
        ]

        num_joints = len(self.joint_state.name)
        time.sleep(3)
        # make sure kit's editor is playing for receiving messages
        self.joint_state.position = np.array([0.0] * num_joints, dtype=np.float64).tolist()
        # self.default_joints = [-0.17378, -1.1635, 0.0485076, -2.29826, -0.108403, 1.59205, 0.978677, 0.4, 0.4]
        # self.default_joints = [0.038, -1.88, 0.001, -8.829, 0.001, 9.5228, 2.46, 0.04, 0.04]
        # self.default_joints = [0.0, -1.16, -0.0, -2.3, -0.0, 1.6, 1.1, 0.4, 0.4]
        self.default_joints = [1.66431, -0.576768, -0.160407, -0.83728, -0.324033, 0.280339, 1.2153, 0.4, 0.4]
        self.joint_state.position = np.array(self.default_joints).tolist()
        self.publisher_.publish(self.joint_state)
        print("pubulished1 = ", self.default_joints)
        time.sleep(5)
        # self.default_joints = [0.0, -1.16, -0.0, -2.3, -0.0, 1.6, 1.1, 0.4, 0.4]

        # self.default_joints = [-0.235428, 6.59728, -2.60891, -0.252463, -1.4431, 0.15886, -4.3075, 0.2, 0.2]
        # self.default_joints = [0.0, 0.71, -0.0, -1.9, -0.0, 2.6, 0.7, 0.2, 0.2]

        # self.default_joints = [-0.301676, 0.617674, -0.170646, 1.61017, -0.200722, 1.48544, 2.58259, 0.2, 0.2]
        # limiting the movements to a smaller range (this is not the range of the robot, just the range of the movement
        # self.max_joints = np.array(self.default_joints) + 0.5
        # self.min_joints = np.array(self.default_joints) - 0.5

        # position control the robot to wiggle around each joint
        # self.joint_state.position = np.array(self.default_joints).tolist()
        # self.publisher_.publish(self.joint_state)
        # print("pubulished2 = ", self.default_joints)

        # time.sleep(3)
        # self.default_joints = [0.0, 0.718, -0.0, -1.9, -0.0, 2.6, 0.7, 0.2, 0.2]
        # self.joint_state.position = np.array(self.default_joints).tolist()
        # self.publisher_.publish(self.joint_state)
        # time.sleep(3)
        # self.default_joints = [0.0, 0.718, -0.0, -1.9, -0.0, 2.6, 0.7, 0.00, 0.00]
        # self.joint_state.position = np.array(self.default_joints).tolist()
        # self.publisher_.publish(self.joint_state)
        # time.sleep(3)
        # self.default_joints = [0.0, 0.62, -0.0, -1.9, -0.0, 2.6, 0.7, 0.00, 0.00]
        # self.joint_state.position = np.array(self.default_joints).tolist()
        # self.publisher_.publish(self.joint_state)




        # self.time_start = time.time()
        # timer_period = 0.05  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    # def timer_callback(self):
    #     self.joint_state.header.stamp = self.get_clock().now().to_msg()

    #     joint_position = (
    #         np.sin(time.time() - self.time_start) * (self.max_joints - self.min_joints) * 0.5 + self.default_joints
    #     )
    #     self.joint_state.position = joint_position.tolist()

    #     # Publish the message to the topic
    #     self.publisher_.publish(self.joint_state)
    #     print("JointState=",self.joint_state.header)


def main(args=None):
    rclpy.init(args=args)

    ros2_publisher = TestROS2Bridge()

    rclpy.spin(ros2_publisher)

    # Destroy the node explicitly
    ros2_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
