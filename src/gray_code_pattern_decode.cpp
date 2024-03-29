// Written and modified by Ammar Nahari axn337@case.edu
// June 2022

#include <ros/ros.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/structured_light.hpp>
#include <opencv2/opencv_modules.hpp>

// (if you did not build the opencv_viz module, you will only see the disparity images)
#ifdef HAVE_OPENCV_VIZ
#include <opencv2/viz.hpp>
#endif

using namespace std;
using namespace cv;

static bool readStringList( const string& filename, vector<string>& l )
{
  l.resize( 0 );
  FileStorage fs( filename, FileStorage::READ );
  if( !fs.isOpened() )
  {
    cerr << "failed to open " << filename << endl;
    return false;
  }
  FileNode n = fs.getFirstTopLevelNode();
  if( n.type() != FileNode::SEQ )
  {
    cerr << "cam 1 images are not a sequence! FAIL" << endl;
    return false;
  }
  FileNodeIterator it = n.begin(), it_end = n.end();
  for( ; it != it_end; ++it )
  {
    l.push_back( ( string ) *it );
  }
  it = n.begin(), it_end = n.end();
  for( ; it != it_end; ++it )
  {
    l.push_back( ( string ) *it );
  }
  if( l.size() % 2 != 0 )
  {
    cout << "Error: the image list contains odd (non-even) number of elements\n";
    return false;
  }
  return true;
}
int main( int argc, char** argv )
{
  ros::init(argc, argv, "gray_code_pattern_decode");
  structured_light::GrayCodePattern::Params params;
  String images_file = "/home/ammarnahari/ros_ws/src/dvrk_structure_light/param/structured_light_data.yaml";
  params.width = 300;
  params.height = 300;

  // Set up GraycodePattern with params
  Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create( params );
  size_t white_thresh = 4;
  size_t black_thresh = 5;
  graycode->setWhiteThreshold( white_thresh );
  graycode->setBlackThreshold( black_thresh );

  vector<string> imagelist;
  bool ok = readStringList( images_file, imagelist );

  if( !ok || imagelist.empty() )
  {
    cerr << "can not open " << images_file << " or the string list is empty" << endl;
    return -1;
  }

  // Loading calibration parameters
  double fx = 1164.636172,	fy = 1169.610140,	 cx = 702.585302,	 cy = 398.066864, k1 = -0.350784,	k2 = 0.286166,	 k3 = 0.319847,	 p1 = -0.000203,	 p2 = 0.006485;

  Mat cam1intrinsics, cam1distCoeffs;
  cam1intrinsics= (Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cam1distCoeffs= (Mat_<double>(1,3) << k1, k2, p1, p2, k3);

  cout << "cam1intrinsics" << endl << cam1intrinsics << endl;
  cout << "cam1distCoeffs" << endl << cam1distCoeffs << endl;
  if((!cam1intrinsics.data) || (!cam1distCoeffs.data))
  {
    cout << "Failed to load cameras calibration parameters" << endl;
    return -1;
  }
  size_t numberOfPatternImages = graycode->getNumberOfPatternImages();
  vector<vector<Mat> > captured_pattern;
  captured_pattern.resize( 2 );
  captured_pattern[0].resize( numberOfPatternImages );
  captured_pattern[1].resize( numberOfPatternImages );
  Mat color = imread( imagelist[numberOfPatternImages], IMREAD_COLOR );
  Size imagesSize = color.size();
  // Stereo rectify
  cout << "Rectifying images..." << endl;
  Mat R1, P1;
  R1 = (Mat_<double>(3,3) << 
    0.999707,	-0.017115,	0.017118,	
    0.017408,	0.999702,	-0.017115,	
    -0.016820,	0.017408,	0.999707);
  cout << "R1" << endl << R1 <<endl;
  Mat map1x, map1y, map2x, map2y;
  // P1= (Mat_<double>(1,3) << -0.056798, 0.078224, -0.053810);
  P1= getOptimalNewCameraMatrix(cam1intrinsics,cam1distCoeffs,imagesSize,1,imagesSize);

  cout << "P1" << endl << P1 <<endl;
  
  initUndistortRectifyMap( cam1intrinsics, cam1distCoeffs, R1, P1, imagesSize, CV_32FC1, map1x, map1y );
  
  cout<< "Done init Undistort" <<endl;

  // Loading pattern images
  for( size_t i = 0; i < numberOfPatternImages; i++ )
  {
    captured_pattern[0][i] = imread( imagelist[i], IMREAD_GRAYSCALE );
    captured_pattern[1][i] = imread( imagelist[i + numberOfPatternImages + 2], IMREAD_GRAYSCALE );
    if( (!captured_pattern[0][i].data) || (!captured_pattern[1][i].data) )
    {
      cerr << "Empty images" << endl;
      return -1;
    }
    remap( captured_pattern[1][i], captured_pattern[1][i], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );

  }
  cout << "done" << endl;
  vector<Mat> blackImages;
  vector<Mat> whiteImages;
  blackImages.resize( 2 );
  whiteImages.resize( 2 );
  // Loading images (all white + all black) needed for shadows computation
  cvtColor( color, whiteImages[0], COLOR_RGB2GRAY );
  whiteImages[1] = imread( imagelist[2 * numberOfPatternImages + 2], IMREAD_GRAYSCALE );
  blackImages[0] = imread( imagelist[numberOfPatternImages + 1], IMREAD_GRAYSCALE );
  blackImages[1] = imread( imagelist[2 * numberOfPatternImages + 2 + 1], IMREAD_GRAYSCALE );
  remap( color, color, map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
  remap( whiteImages[0], whiteImages[0], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
  remap( whiteImages[1], whiteImages[1], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
  remap( blackImages[0], blackImages[0], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
  remap( blackImages[1], blackImages[1], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
  cout << endl << "Decoding pattern ..." << endl;


  Mat disparityMap;
  bool decoded = graycode->decode( captured_pattern, disparityMap, blackImages, whiteImages,
                                  structured_light::DECODE_3D_UNDERWORLD );
  if( decoded )
  {
    cout << endl << "pattern decoded" << endl;
    // To better visualize the result, apply a colormap to the computed disparity
    double min;
    double max;
    minMaxIdx(disparityMap, &min, &max);
    Mat cm_disp, scaledDisparityMap;
    cout << "disp min " << min << endl << "disp max " << max << endl;
    convertScaleAbs( disparityMap, scaledDisparityMap, 255 / ( max - min ) );
    applyColorMap( scaledDisparityMap, cm_disp, COLORMAP_JET );
    // Show the result
    resize( cm_disp, cm_disp, Size( 640, 480 ), 0, 0, INTER_LINEAR );
    imshow( "cm disparity m", cm_disp );
    // Compute the point cloud
    Mat pointcloud;
    disparityMap.convertTo( disparityMap, CV_32FC1 );
    // reprojectImageTo3D( disparityMap, pointcloud, Q, true, -1 );
    // Compute a mask to remove background
    Mat dst, thresholded_disp;
    threshold( scaledDisparityMap, thresholded_disp, 0, 255, THRESH_OTSU + THRESH_BINARY );
    resize( thresholded_disp, dst, Size( 640, 480 ), 0, 0, INTER_LINEAR );
    imshow( "threshold disp otsu", dst );
#ifdef HAVE_OPENCV_VIZ
    // Apply the mask to the point cloud
    Mat pointcloud_tresh, color_tresh;
    pointcloud.copyTo( pointcloud_tresh, thresholded_disp );
    color.copyTo( color_tresh, thresholded_disp );
    // Show the point cloud on viz
    viz::Viz3d myWindow( "Point cloud with color" );
    myWindow.setBackgroundMeshLab();
    myWindow.showWidget( "coosys", viz::WCoordinateSystem() );
    myWindow.showWidget( "pointcloud", viz::WCloud( pointcloud_tresh, color_tresh ) );
    myWindow.showWidget( "text2d", viz::WText( "Point cloud", Point(20, 20), 20, viz::Color::green() ) );
    myWindow.spin();
#endif // HAVE_OPENCV_VIZ
  }
  waitKey();
  return 0;
}