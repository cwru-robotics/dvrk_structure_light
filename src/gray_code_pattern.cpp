#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

static void help()
{
  cout << "\nThis example shows how to use the \"Structured Light module\" to acquire a graycode pattern"
        "\nCall (with the two cams connected):\n"
        "./example_structured_light_cap_pattern <path> <proj_width> <proj_height> \n"
        << endl;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "gray_code_pattern");

  structured_light::GrayCodePattern::Params params;

  // projector resolution
  int proj_height= 500;
  int proj_width=500;

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

// Open camera number 1, using libgphoto2
VideoCapture cap1( CAP_GPHOTO2 );
if( !cap1.isOpened() )
{
  // check if cam1 opened
  cout << "cam1 not opened!" << endl;
  // help();
  return -1;
}
// Turning off autofocus
cap1.set( CAP_PROP_SETTINGS, 1 );

int i = 0;
while( i < (int) pattern.size() )
{
  cout << "Waiting to save image number " << i + 1 << endl << "Press any key to acquire the photo" << endl;
  imshow( "Pattern Window", pattern[i] );
  Mat frame1;
  cap1 >> frame1;  // get a new frame from camera 1
  if( ( frame1.data ) )
  {
    Mat tmp;
    cout << "cam 1 size: " << Size( ( int ) cap1.get( CAP_PROP_FRAME_WIDTH ), ( int ) cap1.get( CAP_PROP_FRAME_HEIGHT ) )
          << endl;
    cout << "zoom cam 1: " << cap1.get( CAP_PROP_ZOOM ) << endl;
    cout << "focus cam 1: " << cap1.get( CAP_PROP_FOCUS ) << endl;
    cout << "Press enter to save the photo or an other key to re-acquire the photo" << endl;
    namedWindow( "cam1", WINDOW_NORMAL );
    resizeWindow( "cam1", 500, 500 );
    // Resizing images to avoid issues for high resolution images, visualizing them as grayscale
    resize( frame1, tmp, Size( 500, 500 ), 0, 0, INTER_LINEAR_EXACT);
    cvtColor( tmp, tmp,CV_RGB2GRAY );
    imshow( "cam1", tmp );
    bool save1 = false;
    int key = waitKey( 0 );
    // Pressing enter, it saves the output
    if( key == 13 )
    {
      ostringstream name;
      name << i + 1;
      save1 =  imwrite( path + "pattern_cam1_im" + name.str() + ".png", frame1 );
      if( ( save1 ) )
      {
        cout << "pattern cam1 images number " << i + 1 << " saved" << endl << endl;
        i++;
      }
      else
      {
        cout << "pattern cam1 images number " << i + 1 << " NOT saved" << endl << endl << "Retry, check the path"<< endl << endl;
      }
    }
    // Pressing escape, the program closes
    if( key == 27 )
    {
      cout << "Closing program" << endl;
    }
  }
  else
  {
    cout << "No frame data, waiting for new frame" << endl;
  }
}
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
