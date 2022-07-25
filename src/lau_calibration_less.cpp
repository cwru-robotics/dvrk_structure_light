#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include <iostream>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

bool got_image1;
cv::Mat img1;

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
void capture_code(String level, std::vector<cv::Mat> pattern, int width, int height)
{

  // Setting pattern window on second monitor (the projector's one)
  namedWindow( "Pattern Window", WINDOW_NORMAL );
  resizeWindow( "Pattern Window", width, height );
  moveWindow( "Pattern Window", 650, 574);

  String path = "/home/ammarnahari/ros_ws/src/dvrk_structure_light/data/";
  int i = 0;
  while( i < (int) pattern.size() )
  {
    imshow( "Pattern Window", pattern[i] );

    waitKey(1);
    // ros::Duration(0.1).sleep();
    ros::spinOnce();

    got_image1 = false;
    int count_image1_cb=0;
    while(!got_image1&&ros::ok){
      ros::spinOnce();
      ROS_WARN_STREAM("Waiting for image 1 data "<<count_image1_cb);
      ros::Duration(0.1).sleep();
      count_image1_cb++;
    }    
    

    Mat frame1;
    frame1=img1;  // get a new frame from camera 1
    if(  frame1.data )
    {
      Mat tmp;

      namedWindow( "cam1", WINDOW_NORMAL );
      resizeWindow( "cam1", 640, 480 );

      // Resizing images to avoid issues for high resolution images, visualizing them as grayscale
      resize( frame1, tmp, Size( 640, 480 ), 0, 0, INTER_LINEAR);
      cvtColor( tmp, tmp, COLOR_RGB2GRAY );
      imshow( "cam1", tmp );
      bool save1 = false;
      ostringstream name;
      name << i + 1;
      save1 = imwrite( path + level +"/im" + name.str() + ".png", frame1 );
      if( save1 )
      {
        cout << "pattern "+level+ " images number " << i + 1 << " saved" << endl << endl;
        i++;
      }
      else
      {
        cout << "pattern "+level+ " images number " << i + 1 << " NOT saved" << endl << endl << "Retry, check the path"<< endl << endl;
      }
    }
    else
    {
      cout << "No frame data, waiting for new frame" << endl;
    }
  }

  destroyAllWindows();
}

int main( int argc, char** argv)
{
  ros::init(argc, argv, "lau_calibration_less");
  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe("/davinci_endo/left/image_raw", 1, image1CB);

  structured_light::GrayCodePattern::Params params;
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

  std::cout<<"r1 acquistion, place board in center of camera view and press enter"<<std::endl;
  std::cin.ignore();

  capture_code("r1",pattern,params.width,params.height);

  std::cout<<"r11 acquistion, raise the board 30mm using blocks and press enter"<<std::endl;
  std::cin.ignore();

  capture_code("r11",pattern,params.width,params.height);

  std::cout<<"r12 acquistion, raise the board 60mm using blocks and press enter"<<std::endl;
  std::cin.ignore();

  capture_code("r12",pattern,params.width,params.height);

  std::cout<<"r2 acquistion, raise the board 90mm using blocks and press enter"<<std::endl;
  std::cin.ignore();

  capture_code("r2",pattern,params.width,params.height);

  std::cout<<"object acquistion, remove the board and blocks and press enter"<<std::endl;
  std::cin.ignore();

  capture_code("obj",pattern,params.width,params.height);

  std::cout<<"Thank you"<<std::endl;

  return 0;

}
