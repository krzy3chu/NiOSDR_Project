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
        self._control_msg = Twist()  # message to send in publisher
        self._dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)  # define aruco type

    def camera_callback(self, image_data):
        cv_image = CvBridge().imgmsg_to_cv2(image_data,"bgr8")
        image_height = cv_image.shape[0]
        image_width = cv_image.shape[1]
        turtle_linear = 0.0 # turtlebot linear velocity
        turtle_angular = 0.0 # turtlebot angular velocity

        # detect aruco markers
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(cv_image, self._dictionary)
        if markerCorners:
            # draw lines around aruco marker
            marker = np.squeeze(markerCorners).astype(int)
            cv2.polylines(cv_image,[marker],True,(0,0,255), 5)
            # draw circle of aruco marker
            M = cv2.moments(marker)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image,(cx, cy), 10, (255,0,0), 5)
            # calculate turtlebot velocity and angle
            turtle_linear = ((image_height // 2) - cy) / 100.0
            turtle_angular = - ((image_width // 2) - cx) / 100.0

        # draw center lines
        cv2.line(cv_image, (0, image_height//2), (image_width, image_height//2), (0,0,0), 3)
        cv2.line(cv_image, (image_width//2, 0), (image_width//2, image_height), (0,0,0), 3)
        # mirror image
        cv_image = cv2.flip(cv_image, 2)
        # show processed camera image
        cv2.imshow("camera", cv_image)
        cv2.waitKey(1)

        # publish and log message
        self._control_msg.linear.x = turtle_linear
        self._control_msg.angular.z = turtle_angular
        self._publisher.publish(self._control_msg)
        self.get_logger().info('Publishing: "%s"' % self._control_msg)


def main(args=None):
    rclpy.init(args=args)

    # handle ros node
    controller_node = ControllerNode()
    controller_node.get_logger().info('Turtle controller node started')
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
