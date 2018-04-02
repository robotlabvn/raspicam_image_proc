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

class center_contour():
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
            kernel = np.ones((5,5),np.float32)/25
            img_filter=cv2.filter2D(grey,-1, kernel)
            ret,thresh = cv2.threshold(img_filter, 90, 255, cv2.THRESH_BINARY_INV)
            contours, hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            cnt = contours[0]
            cv2.drawContours(frame, contours, -1, (0,255,0), 3)
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cX, cY), 7, (0, 0, 255), -1)
            cv2.putText(frame, "center", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return frame
 
    def cleanup(self):
            print "shouting down of Vision Node"
            cv2.destroyAllWindowns()
        
if __name__== "__main__":
        try:
          node_name = "center_contour"
          center_contour(node_name)
          rospy.spin()
        except KeyboardInterrupt:
          print("Shutting down")
          cv2.destroyAllWindows()
