#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <iostream>

bool got_image;
cv::Mat img;

void imageCB(const sensor_msgs::Image::ConstPtr& im)
{
  ROS_INFO("callback");

  cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(im);
		img = cv_ptr->image;
		got_image = true;
		return;
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from encoding to 'bgr8'.");
		return;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_subscriber");
  ros::NodeHandle nh;
  // cv::namedWindow("view");

  cv::namedWindow("view",cv::WINDOW_AUTOSIZE);
  // cv::moveWindow("view",1400,50);

  ROS_INFO("created view  window");


  //image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("/amscope/image_raw", 1, imageCB);
  ros::Subscriber sub = nh.subscribe("/amscope/image_raw", 1, imageCB);

  ROS_INFO("created subscribier, spinning...");

  while (ros::ok)
  {
    got_image = false;
    while(!got_image&&ros::ok){
      ros::spinOnce();
      ROS_WARN("Waiting for image data on topic");
      ros::Duration(0.1).sleep();
    }

    if(img.empty())
    {
      ROS_ERROR("Could not convert the image");
      return 1;
    }

    cv::imshow("view", img);

    cv::waitKey();
  }
  



  cv::destroyWindow("view");
  return 0;
}