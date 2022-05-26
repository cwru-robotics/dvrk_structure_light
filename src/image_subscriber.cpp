#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <iostream>

cv::Mat g_image;

void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("callback");
  cv::Mat image;

  cv_bridge::toCvShare(msg, "bgr8")->image;
  // cv::imshow("view", image);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_subscriber");
  ros::NodeHandle nh;
  // cv::namedWindow("view");

  cv::namedWindow("view",cv::WINDOW_AUTOSIZE);
  // cv::moveWindow("view",1400,50);

  ROS_INFO("created view  window");


  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("pattern_display", 1, imageCB);

  ROS_INFO("created subscribier, spinning...");

  while (ros::ok)
  {
    //  ros::spin();
    ros::spinOnce();

    if(g_image.empty())
    {
      ROS_ERROR("Could not convert the image");
      return 1;
    }

    cv::imshow("view", g_image);

    cv::waitKey();
  }
  



  cv::destroyWindow("view");
  return 0;
}