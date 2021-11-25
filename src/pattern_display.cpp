#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pattern_display");
  ros::NodeHandle nh;
  //cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("pattern_display", 1, imageCallback);

  cv::Mat image1,image2;

  std::string img_path="/home/ammarnahari/ros_ws/src/dvrk_structure_light/img/black_white_stripes.jpg";
  
  image1=cv::imread(img_path,cv::IMREAD_COLOR);

  if(image1.empty())
  {
    ROS_INFO_STREAM("Could not read the image: " << img_path);
    return 1;
  }

  cv::namedWindow("pattern_out",cv::WINDOW_AUTOSIZE);

  cv::imshow("pattern_out",image1);
  
  cv::waitKey(0);

  cv::destroyWindow("Car");

  //ros::spinOnce();
  // cv::destroyWindow("view");
  return 0;
}