#include <ros/ros.h>
// #include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <iostream>

void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("callback");
  cv::Mat image;

  cv_bridge::toCvShare(msg, "bgr8")->image;

  if(image.empty())
  {
    ROS_ERROR("Could not convert the image");
    ros::shutdown();
  }
  try
  {
    cv::imshow("view", image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_subscriber");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  //cv::namedWindow("view",cv::WINDOW_AUTOSIZE);
  cv::moveWindow("view",1400,50);

  ROS_INFO("created view  window");


  // image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("pattern_display", 1, &imageCB);

  ROS_INFO("created subscribier, spinning...");

  ros::spin();
  cv::destroyWindow("view");
}