#!/usr/bin/env python
import rospy
import sys
import cv2
import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
import numpy as np

scale =1
delta = 0
ddepth = cv2.CV_16S

class canny_operator():
    def __init__(self,node_name):
            self.node_name= node_name
            rospy.init_node(self.node_name)
            rospy.on_shutdown(self.cleanup)
        
            self.cv_window_name=self.node_name
            cv.NamedWindow(self.cv_window_name,cv.CV_WINDOW_NORMAL)
            cv.MoveWindow(self.cv_window_name, 400, 75)

            self.bridge = CvBridge()
            self.image_sub=rospy.Subscriber("input_rgb_image", Image, self.callback,queue_size=1)
        
            rospy.loginfo("Waiting for image topic...")
            rospy.wait_for_message("input_rgb_image", Image)
            rospy.loginfo("Ready")
        
    def callback(self, ros_image):
            try:
              frame=self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            except CvBridgeError, e:
              print e

            frame = np.array(frame, dtype =np.uint8)
            display_process_image = self.process_image(frame)
            # Display the process image.
            cv2.imshow(self.node_name, display_process_image)

            self.keystroke =cv2.waitKey(5)
            if self.keystroke != -1:
              cc=chr(self.keystroke & 255).lower()
              if cc == 'q':
                      rospy.signal_shutdown("User hit the q key to quit")
            
    def process_image(self, frame):
# Convert to greyscale
            grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
# Blur the image
            blur = cv2.blur(grey, (3, 3))
# Compute edges using the Canny edge filter
            edges = cv2.Canny(blur, 30.0, 50.0)
            return edges
 
    def cleanup(self):
            print "shouting down of Vision Node"
            cv2.destroyAllWindowns()
        
if __name__== "__main__":
        try:
          node_name = "canny_operator"
          canny_operator(node_name)
          rospy.spin()
        except KeyboardInterrupt:
          print("Shutting down")
          cv2.destroyAllWindows()
