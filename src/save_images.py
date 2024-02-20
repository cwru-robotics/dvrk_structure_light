#! /usr/bin/python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        time = msg.header.stamp
        cv2.imwrite('img1-'+str(time)+'.jpeg', cv2_img)
        print("Received an image! "+str(time))
        rospy.sleep(1)
        
def image2_callback(msg):
    # print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        time = msg.header.stamp
        cv2.imwrite('img2-'+str(time)+'.jpeg', cv2_img)
        rospy.sleep(1)        

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/davinci_endo/left/image_color"
    image2_topic = "/davinci_endo/right/image_color"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber(image2_topic, Image, image2_callback)
    # Spin until ctrl + c
    rospy.spinOnce()

if __name__ == '__main__':
    main()
