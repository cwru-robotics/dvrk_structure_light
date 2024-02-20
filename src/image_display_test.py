#!/usr/bin/env python3

import rospy
import cv2

def main():
    rospy.init_node("pattern_display")

    size=100

    img_path = "/home/dvrk/ros_ws/src/dvrk_structure_light/doc/checkerboard.jpg"  # add path to the image
    # image1 = cv2.resize(cv2.imread(img_path, cv2.IMREAD_COLOR),(size,size))
    image1 = cv2.imread(img_path, cv2.IMREAD_COLOR)


    if image1 is None:
        rospy.loginfo("Could not read the image: %s", img_path)
        return 1

    cv2.namedWindow("pattern_out", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("pattern_out", size, size)
    cv2.moveWindow("pattern_out", 1000, 450) #1280x800/1024


    cv2.imshow("pattern_out", image1)
    cv2.waitKey(0)

    cv2.destroyWindow("pattern_out")

if __name__ == '__main__':
    main()