#include <ros/ros.h>
#include <opencv2/structured_light.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>


using namespace cv;
using namespace std;

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

int main( int argc, char** argv )
{
  ros::init(argc, argv, "gray_code_pattern_capture");

  structured_light::GrayCodePattern::Params params; 

  // projector resolution
  int proj_height= 300;
  int proj_width= 300;

  string path="/home/ammarnahari/ros_ws/src/dvrk_structure_light/img/";

  params.height=proj_height;
  params.width=proj_width;

  // Set up GraycodePattern with params
  Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create( params );
  // Storage for pattern
  vector<Mat> pattern;
  graycode->generate( pattern );

  // Generate the all-white and all-black images needed for shadows mask computation
  Mat white;
  Mat black;
  graycode->getImagesForShadowMasks( black, white );
  pattern.push_back( white );
  pattern.push_back( black );

  // Setting pattern window on projector's monitor
  namedWindow("Pattern Window",WINDOW_NORMAL);
  resizeWindow("Pattern Window",params.width
  ,params.height);
  moveWindow("Pattern Window",600,500);

  // subscribe to amscope driver
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/amscope/image_raw", 1, imageCB);
  ROS_INFO("created subscribier, spinning...");


  //grap and write loop
  int i = 0;
  while( i < (int) pattern.size() )
  {
    cout << "Display pattern " << i << endl;

    imshow( "Pattern Window", pattern[i] );
    got_image = false;

    int count_image_cb=0;
    while(!got_image&&ros::ok){
      ros::spinOnce();
      ROS_WARN_STREAM("Waiting for image data "<<count_image_cb);
      ros::Duration(0.1).sleep();
      count_image_cb++;
    }

    if(img.empty())
    {
      ROS_ERROR("Could not convert the image");
      return 1;
    }


    // imshow("view",img);

    std::string savingName = "/home/ammarnahari/ros_ws/src/dvrk_structure_light/data/" + std::to_string(i) + ".jpg";
    imwrite(savingName,img);
    if (waitKey(1) <= 0) break;
    
    i++;
  }
  return 0;
}
