#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from copy import deepcopy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class GarmentMotionTest(Node):
    
    def __init__(self):
        super().__init__('garment_motion_test')
        print("1")
        # ROS initial
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.current_pose_callback,
            10)
    
    def current_pose_callback(self, msg):
        print("2")
        self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        print("3")
        cv2.imshow('cv_img', self.image)
        print("4")
        cv2.waitKey(10)
    
    def run(self):
        while rclpy.ok():
            print("run")
            rclpy.spin_once(self)
            
            
            

def main(args=None):
    print("0")
    rclpy.init(args=args)
    garment_motion_test = GarmentMotionTest()
    garment_motion_test.run()
    garment_motion_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()