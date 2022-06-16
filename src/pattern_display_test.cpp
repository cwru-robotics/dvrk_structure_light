#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pattern_display");

  cv::Mat image1;

  std::string img_path="/home/ammarnahari/ros_ws/src/dvrk_structure_light/data/Structured-light-pattern_Q640.jpg";
  
  image1=cv::imread(img_path,cv::IMREAD_COLOR);

  if(image1.empty())
  {
    ROS_INFO_STREAM("Could not read the image: " << img_path);
    return 1;
  }
  cv::namedWindow( "pattern_out", cv::WINDOW_NORMAL );
  cv::resizeWindow( "pattern_out", 170, 170 );
  cv::moveWindow("pattern_out",650,574);

  cv::imshow("pattern_out",image1);
  
  cv::waitKey(0);

  cv::destroyWindow("pattern_out");

  return 0;
}