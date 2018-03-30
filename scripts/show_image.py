#!/usr/bin/env python

import rospy
import sys
import cv2
import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class show_image():

    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)

        self.cv_window_name =self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.callback, queue_size=1)

        rospy.loginfo("Waiting for image topics...")
        rospy.wait_for_message("input_rgb_image", Image)
        rospy.loginfo("Ready.")

    def callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        frame = np.array(frame, dtype=np.uint8)
        cv2.imshow(self.node_name, frame)

        self.keystroke = cv2.waitKey(5)
        if self.keystroke != -1:
            cc = chr(self.keystroke & 255).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

    def cleanup(self):
        print "Shutting down of Vision node"
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node_name = "show_image"
        show_image(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
        cv2.DestroyAllWindows()



