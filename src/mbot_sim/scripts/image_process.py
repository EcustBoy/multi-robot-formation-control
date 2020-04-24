#!/usr/bin/env python
import math
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError


class ImgProcess(object):
    def __init__(self):
        self.node_name = 'img_process'

        rospy.init_node(self.node_name)

        self.bridge = CvBridge()

        self.sub1 = rospy.Subscriber("rgb/image_raw", Image, self.img_process_callback)
        self.pub1=rospy.Publisher("rgb/image_processed",Image,queue_size=10)
        rospy.loginfo("Waiting for image topics...")

    def image_process(self, image):
    	
        image_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
		
        orange_min = np.array([100, 128, 46])
        orange_max = np.array([124, 255, 255])
        mask = cv2.inRange(image_hsv, orange_min, orange_max)
        ret, binary = cv2.threshold(mask, 1, 256, cv2.THRESH_BINARY)

        non0_point = 0
        non0_rows = 0
        non0_cols = 0
        length_max = 0
        width_max = 0
        length_min = 0
        width_min = 0
        point_exist = False
	
        rows, cols = np.shape(binary)
        for m in range(rows):
            for n in range(cols):
                if binary[m, n] != 0:
          
                    non0_rows += m
                    non0_cols += n
                    non0_point += 1
							
                    if non0_point == 1:
                        width_min = n
                        length_min = m
                    if n < width_min:
                        width_min = n
                    if n > width_max:
                        width_max = n
                    if m < length_min:
                        length_min = m
                    if m > length_max:
                        length_max = m
                        
        if non0_point != 0:
            point_exist = True
            midpoint = [int(non0_rows / non0_point), int(non0_cols / non0_point)]
            min_point = [length_min, width_min]
            max_point = [length_max, width_max]
        else:
            midpoint = 0
            min_point = 0
            max_point = 0

        res = cv2.bitwise_and(image, image, mask=mask)
        cv2.circle(res, (midpoint[1], midpoint[0]), 5, (0, 0, 255), 2)
        cv2.rectangle(res, (min_point[1], min_point[0]), (max_point[1], max_point[0]), (0, 0, 255))

        return res, midpoint, min_point, max_point, point_exist
        
    def img_process_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError:
            rospy.loginfo('CvBrigeError!')

        frame = np.array(frame,dtype=np.uint8)
        processed_image, midpoint, min_point, max_point, point_exist = self.image_process(frame)
        processed_Image = self.bridge.cv2_to_imgmsg(processed_image,"bgr8")
        cv2.imshow('image', processed_Image)
        if point_exist:
            rospy.loginfo('midpoint: %s, image_range: %s, %s', midpoint, min_point, max_point)
    
            processed_Image = self.bridge.cv2_to_imgmsg(processed_image,"bgr8")
          
            self.pub1.publish(processed_Image)
            cv2.imshow('image', processed_Image)
        else:
            rospy.loginfo('No targets were detected')


if __name__ == '__main__':
    try:
        ImgProcess()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
  
