#!/usr/bin/env python3

import rospy
import cv2

def main():
    rospy.init_node("pattern_display")

    img_path = "/home/axn337/Downloads/black_white_stripes.jpg"  # add path to the image
    image1 = cv2.imread(img_path, cv2.IMREAD_COLOR)

    if image1 is None:
        rospy.loginfo("Could not read the image: %s", img_path)
        return 1

    cv2.namedWindow("pattern_out", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("pattern_out", 500, 500)
    cv2.moveWindow("pattern_out", 750, 574)

    cv2.imshow("pattern_out", image1)
    cv2.waitKey(0)

    cv2.destroyWindow("pattern_out")

if __name__ == '__main__':
    main()