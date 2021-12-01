#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "gray_code_pattern_out");

  structured_light::GrayCodePattern::Params params;

  // projector resolution
  int proj_height= 400;
  int proj_width= 400;

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

  // Setting pattern window on second monitor (the projector's one)
  namedWindow("Pattern Window",WINDOW_NORMAL);
  resizeWindow("Pattern Window",params.width
  ,params.height);
  moveWindow("Pattern Window",1920+150,500);

  int i = 0;
  while( i < (int) pattern.size() )
  {
    cout << "Waiting to save image number " << i + 1 << endl << "Press any key to acquire the photo" << endl;
    imshow( "Pattern Window", pattern[i] );
    cv::waitKey(0);
    i++;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
