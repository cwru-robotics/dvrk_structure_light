/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2015, OpenCV Foundation, all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include <iostream>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
bool got_image1,got_image2;
cv::Mat img1,img2;
void image1CB(const sensor_msgs::Image::ConstPtr& im)
{
  ROS_INFO("callback");

  cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(im);
		img1 = cv_ptr->image;
		got_image1 = true;
		return;
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from encoding to 'bgr8'.");
		return;
	}
}
void image2CB(const sensor_msgs::Image::ConstPtr& im)
{
  ROS_INFO("callback");

  cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(im);
		img2 = cv_ptr->image;
		got_image2 = true;
		return;
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from encoding to 'bgr8'.");
		return;
	}
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "stereo_gray_code_pattern_capture");
  structured_light::GrayCodePattern::Params params;
  String path = "/home/ammarnahari/ros_ws/src/dvrk_structure_light/data/";
  params.width = 300;
  params.height = 300;

  // Set up GraycodePattern with params
  Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create( params );
  // Storage for pattern
  vector<Mat> pattern;
  graycode->generate( pattern );
  cout << pattern.size() << " pattern images + 2 images for shadows mask computation to acquire with both cameras"
         << endl;
  // Generate the all-white and all-black images needed for shadows mask computation
  Mat white;
  Mat black;
  graycode->getImagesForShadowMasks( black, white );
  pattern.push_back( white );
  pattern.push_back( black );

  cout << pattern.size() << " total patterns with shadow masks white average:"<< mean(pattern[pattern.size()-1]) <<" black average: "<<mean(pattern[pattern.size()-2])
         << endl;

  // Setting pattern window on second monitor (the projector's one)
  namedWindow( "Pattern Window", WINDOW_NORMAL );
  resizeWindow( "Pattern Window", params.width, params.height );
  moveWindow( "Pattern Window", 650, 574);
  //setWindowProperty( "Pattern Window", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN );
  // subscribe to dvrk cams
  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe("/davinci_endo/left/image_raw", 1, image1CB);
  ros::Subscriber sub2 = nh.subscribe("/davinci_endo/right/image_raw", 1, image2CB);

  ROS_INFO("created subscribiers, spinning...");

  int i = 0;
  while( i < (int) pattern.size() )
  {
    cout << "Waiting to save image number " << i + 1 << endl << "Press any key to acquire the photo" << endl;
    imshow( "Pattern Window", pattern[i] );

    waitKey(0);
    

    ros::spinOnce();

    got_image1 = false;
    int count_image1_cb=0;
    while(!got_image1&&ros::ok){
      ros::spinOnce();
      ROS_WARN_STREAM("Waiting for image 1 data "<<count_image1_cb);
      ros::Duration(0.1).sleep();
      count_image1_cb++;
    }    
    
    got_image2 = false;

    int count_image2_cb=0;
    while(!got_image2&&ros::ok){
      ros::spinOnce();
      ROS_WARN_STREAM("Waiting for image 2 data "<<count_image2_cb);
      ros::Duration(0.1).sleep();
      count_image2_cb++;
    }

    Mat frame1;
    Mat frame2;
    frame1=img1;  // get a new frame from camera 1
    frame2=img2;  // get a new frame from camera 2

    if( ( frame1.data ) && ( frame2.data ) )
    {
      Mat tmp;
      // cout << "cam 1 size: " << Size( ( int ) cap1.get( CAP_PROP_FRAME_WIDTH ), ( int ) cap1.get( CAP_PROP_FRAME_HEIGHT ) )
      //      << endl;
      // cout << "cam 2 size: " << Size( ( int ) cap2.get( CAP_PROP_FRAME_WIDTH ), ( int ) cap2.get( CAP_PROP_FRAME_HEIGHT ) )
      //      << endl;
      // cout << "zoom cam 1: " << cap1.get( CAP_PROP_ZOOM ) << endl << "zoom cam 2: " << cap2.get( CAP_PROP_ZOOM )
      //      << endl;
      // cout << "focus cam 1: " << cap1.get( CAP_PROP_FOCUS ) << endl << "focus cam 2: " << cap2.get( CAP_PROP_FOCUS )
      //      << endl;
      namedWindow( "cam1", WINDOW_NORMAL );
      resizeWindow( "cam1", 640, 480 );
      namedWindow( "cam2", WINDOW_NORMAL );
      resizeWindow( "cam2", 640, 480 );
      // Moving window of cam2 to see the image at the same time with cam1
      moveWindow( "cam2", 640 + 75, 0 );
      // Resizing images to avoid issues for high resolution images, visualizing them as grayscale
      resize( frame1, tmp, Size( 640, 480 ), 0, 0, INTER_LINEAR);
      cvtColor( tmp, tmp, COLOR_RGB2GRAY );
      imshow( "cam1", tmp );
      resize( frame2, tmp, Size( 640, 480 ), 0, 0, INTER_LINEAR);
      cvtColor( tmp, tmp, COLOR_RGB2GRAY );
      imshow( "cam2", tmp );
      bool save1 = false;
      bool save2 = false;
      ostringstream name;
      name << i + 1;
      save1 = imwrite( path + "pattern_cam1_im" + name.str() + ".png", frame1 );
      save2 = imwrite( path + "pattern_cam2_im" + name.str() + ".png", frame2 );
      if( ( save1 ) && ( save2 ) )
      {
        cout << "pattern cam1 and cam2 images number " << i + 1 << " saved" << endl << endl;
        i++;
      }
      else
      {
        cout << "pattern cam1 and cam2 images number " << i + 1 << " NOT saved" << endl << endl << "Retry, check the path"<< endl << endl;
      }
      // if (waitKey(1) <= 0) break;
      // waitKey(0);
    }
    else
    {
      cout << "No frame data, waiting for new frame" << endl;
    }
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}