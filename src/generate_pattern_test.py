#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

def main():
    rospy.init_node("pattern_display")

    size=100

    # Set the dimensions of the pattern (100x100)
    width, height = 100, 100

    # Create a blank white image
    pattern_image = np.ones((height, width), dtype=np.uint8) * 255

    # Draw a black checkerboard pattern
    for i in range(height):
        for j in range(width):
            if (i + j) % 2 == 0:
                pattern_image[i, j] = 0

    cv2.namedWindow("pattern_out", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("pattern_out", size, size)
    cv2.moveWindow("pattern_out", 1000, 450) #1280x800/1024


    cv2.imshow("pattern_out", pattern_image)
    cv2.waitKey(0)

    cv2.destroyWindow("pattern_out")

if __name__ == '__main__':
    main()