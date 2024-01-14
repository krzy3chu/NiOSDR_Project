#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self._subscription = self.create_subscription(Image, 'image_raw', self.camera_callback, 10)
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self._control_msg = Twist()

    def camera_callback(self, image_data):
        cv_image = CvBridge().imgmsg_to_cv2(image_data,"bgr8")
        image_height = cv_image.shape[0]
        image_width = cv_image.shape[1]
        turtle_velocity = 0.0

        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(cv_image, self._dictionary)
        if markerCorners:
            marker = np.squeeze(markerCorners).astype(int)
            cv2.polylines(cv_image,[marker],True,(0,0,255), 5)
            M = cv2.moments(marker)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image,(cx, cy), 10, (255,0,0), 5)
            turtle_velocity = ((image_height / 2) - cy) / 100.0

        cv2.line(cv_image, (0, int(image_height/2)), (image_width, int(image_height/2)), (0,0,0), 3)
        cv2.imshow("camera", cv_image)
        cv2.waitKey(1)

        print(type(turtle_velocity))
        self._control_msg.linear.x = turtle_velocity
        self._publisher.publish(self._control_msg)
        self.get_logger().info('Publishing: "%s"' % self._control_msg)



def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()
    controller_node.get_logger().info('Turtle controller node started')

    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()