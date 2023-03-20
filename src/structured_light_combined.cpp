
#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include <iostream>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/structured_light.hpp>
#include <opencv2/opencv_modules.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;

bool got_image1,got_image2;
cv::Mat img1,img2;

ros::Publisher image_pub;
ros::Subscriber sub1;
ros::Subscriber sub2;

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
  n = fs["cam2"];
  if( n.type() != FileNode::SEQ )
  {
    cerr << "cam 2 images are not a sequence! FAIL" << endl;
    return false;
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

void intrinsic_file_read(string file_name, Mat* intrinsics, Mat* distCoeffs){
  
  // cout<<"opening file "<<file_name<<endl;
  ifstream inFil(file_name);

  // Loading calibration parameters
  double fx,fy,cx,cy,k1,k2,k3,p1,p2;

  string temp;
  double found;
  vector<double> data_list;
  data_list.clear();
  while (inFil)
  {
    inFil >> found;
    data_list.push_back(found);
  }
  
  fx=data_list[0];
  fy=data_list[1];
  cx=data_list[2];
  cy=data_list[3];
  k1=data_list[4];
  k2=data_list[5];
  k3=data_list[6];
  p1=data_list[7];
  p2=data_list[8];


  // ROS_INFO("fx %lf, cx %lf, fy %lf, cy %lf, k1 %lf, k2 %lf, p1 %lf, p2 %lf, k3 %lf",fx, cx, fy, cy, k1, k2, p1, p2, k3);
  

  *intrinsics= (Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  *distCoeffs= (Mat_<double>(1,5) << k1, k2, p1, p2, k3);

}
void reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n)
{
  for (size_t i=0; i<n; ++i)
    dst[n-1-i] = src[i];
}

void cv_to_ros_pointcloud(cv::Mat mat,Mat color)
{
  // PointCloud2Ptr points_msg=boost::make_shared<PointCloud2>();
  std_msgs::Header hdr;
  int seq=0;
  string image_frame="structured_light_camera";
  while (ros::ok)
  {
    // ros pointcloud processing influenced by https://gist.github.com/plusk01/15888d87c25498160cde1424e4c8271d

    sensor_msgs::PointCloud2 cloudmsg;
    cloudmsg.header.stamp = ros::Time::now();
    cloudmsg.header.frame_id = image_frame;
    cloudmsg.width = mat.cols;
    cloudmsg.height = mat.rows;
    cloudmsg.is_bigendian =false;
    cloudmsg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloudmsg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    std::string format_str = "rgb";

    cloudmsg.point_step = addPointField(cloudmsg, format_str.c_str(), 1, sensor_msgs::PointField::UINT32, cloudmsg.point_step);
    cloudmsg.row_step = cloudmsg.width * cloudmsg.point_step;
    cloudmsg.data.resize(cloudmsg.height * cloudmsg.row_step);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloudmsg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloudmsg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloudmsg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_color(cloudmsg, format_str);

    uint8_t* color_data = color.data;//.data;

    int num_colors = 3;

    for (size_t r=0; r<mat.rows; ++r) {
      for (size_t c=0; c<mat.cols; ++c) {

        *iter_x = c*0.010;
        *iter_y = r*0.010;
        *iter_z = *iter_x + *iter_y;
        int offset = (r * mat.cols + c) * num_colors;
        reverse_memcpy(&(*iter_color), color_data+offset, num_colors);

        ++iter_x; ++iter_y; ++iter_z;
        ++iter_color;
      }
    }

    image_pub.publish(cloudmsg);
    seq++;
    ros::Duration(0.1).sleep();
  }
}


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

int gray_code_pattern_capture()
{
  ROS_INFO("Gray Code Capture");

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

  ROS_INFO("created subscribiers, spinning...");

  int i = 0;
  while( i < (int) pattern.size() )
  {
    cout << "Waiting to save image number " << i + 1 << endl << "Press any key to acquire the photo" << endl;
    imshow( "Pattern Window", pattern[i] );

    waitKey(250);
    

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
  destroyAllWindows();
  return 0;
}

int gray_code_pattern_decode()
{
  ROS_INFO("Gray Code Decode");
    
  structured_light::GrayCodePattern::Params params;
  String images_file ="/home/ammarnahari/ros_ws/src/dvrk_structure_light/param/stereo_structured_light_data.yaml";
  String cam1_intrinnsics_file="/home/ammarnahari/ros_ws/src/dvrk_structure_light/param/left_cam_intrinsics.txt";
  String cam2_intrinsics_file="/home/ammarnahari/ros_ws/src/dvrk_structure_light/param/right_cam_intrinsics.txt";
  
  params.width = 300;
  params.height = 300;

  // Set up GraycodePattern with params
  Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create( params );
  size_t white_thresh = 0;
  size_t black_thresh = 0;

  // If passed, setting the white and black threshold, otherwise using default values
  white_thresh =145;
  black_thresh = 190;
  graycode->setWhiteThreshold( white_thresh );
  graycode->setBlackThreshold( black_thresh );
  
  vector<string> imagelist;
  bool ok = readStringList( images_file, imagelist );
  if( !ok || imagelist.empty() )
  {
    cout << "can not open " << images_file << " or the string list is empty" << endl;
    return -1;
  }

  Mat cam1intrinsics, cam1distCoeffs, cam2intrinsics, cam2distCoeffs, R, T;

  intrinsic_file_read(cam1_intrinnsics_file,&cam1intrinsics,&cam1distCoeffs);

  intrinsic_file_read(cam2_intrinsics_file,&cam2intrinsics,&cam2distCoeffs);
  R= (
    Mat_<double>(3,3)<<
     1.0000, 0.0002,-0.0002,
    -0.0002, 1.0000, 0.0002,
     0.0002,-0.0002, 1.0000  
  );
  T= (Mat_<double>(3,1) << 0.0056858,-0.0003983, 0.0003763);
  // cout << "cam1intrinsics" << endl << cam1intrinsics << endl;
  // cout << "cam1distCoeffs" << endl << cam1distCoeffs << endl;
  // cout << "cam2intrinsics" << endl << cam2intrinsics << endl;
  // cout << "cam2distCoeffs" << endl << cam2distCoeffs << endl;
  // cout << "T" << endl << T << endl << "R" << endl << R << endl;

  // return 0;

  if( (!R.data) || (!T.data) || (!cam1intrinsics.data) || (!cam2intrinsics.data) || (!cam1distCoeffs.data) || (!cam2distCoeffs.data) )
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
  Mat R1, R2, P1, P2, Q;
  Rect validRoi[2];
  stereoRectify( cam1intrinsics, cam1distCoeffs, cam2intrinsics, cam2distCoeffs, imagesSize, R, T, R1, R2, P1, P2, Q, 0,
                -1, imagesSize, &validRoi[0], &validRoi[1] );
  Mat map1x, map1y, map2x, map2y;
  
  cout << "initUndistortRectifyMap..." << endl;
  initUndistortRectifyMap( cam1intrinsics, cam1distCoeffs, R1, P1, imagesSize, CV_32FC1, map1x, map1y );
  initUndistortRectifyMap( cam2intrinsics, cam2distCoeffs, R2, P2, imagesSize, CV_32FC1, map2x, map2y );

  // cout<<"R1"<<endl<<R1<<"\nR2\n"<<R2<<"\nP1\n"<<P1<<"\nP2\n"<<P2<<"\nQ\n"<<Q<<"\nmap1x\n"<<map1x.size()<<"\nmap1y\n"<<map1y.size()<<"\nmap2x\n"<<map2x.size()<<"\nmap2y\n"<<map2y.size()<<endl;
  // Loading pattern images
  for( size_t i = 0; i < numberOfPatternImages; i++ )
  {
    captured_pattern[0][i] = imread( imagelist[i], IMREAD_GRAYSCALE );
    captured_pattern[1][i] = imread( imagelist[i + numberOfPatternImages + 2], IMREAD_GRAYSCALE );
    if( (!captured_pattern[0][i].data) || (!captured_pattern[1][i].data) )
    {
      cout << "Empty image " <<i<< endl;
      return -1;
    }
    remap( captured_pattern[1][i], captured_pattern[1][i], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
    remap( captured_pattern[0][i], captured_pattern[0][i], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar() );
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
    namedWindow("cm disparity m",WINDOW_AUTOSIZE);
    moveWindow("cm disparity m",1600,500);
    imshow( "cm disparity m", cm_disp );
    // Compute the point cloud
    Mat pointcloud;
    disparityMap.convertTo( disparityMap, CV_32FC1 );
    reprojectImageTo3D( disparityMap, pointcloud, Q, true, -1 );
    // Compute a mask to remove background
    Mat dst, thresholded_disp;
    threshold( scaledDisparityMap, thresholded_disp, 0, 255, THRESH_OTSU + THRESH_BINARY );
    resize( thresholded_disp, dst, Size( 640, 480 ), 0, 0, INTER_LINEAR );
    namedWindow("threshold disp otsu",WINDOW_AUTOSIZE);
    moveWindow("threshold disp otsu",1280,0);
    imshow( "threshold disp otsu", dst );

    // Apply the mask to the point cloud
    Mat pointcloud_tresh, color_tresh;
    pointcloud.copyTo( pointcloud_tresh, thresholded_disp );
    color.copyTo( color_tresh, thresholded_disp );
    waitKey();

    cv_to_ros_pointcloud(pointcloud,color);
  }

  
  return 0;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "structured_light_combined");
  ros::NodeHandle(nh);

  sub1 = nh.subscribe("/davinci_endo/left/image_raw", 1, image1CB);
  sub2 = nh.subscribe("/davinci_endo/right/image_raw", 1, image2CB);

  image_pub = nh.advertise<PointCloud2>("dvrk_structured_light_pointcloud",1);

  int capture_success=gray_code_pattern_capture();
  cout<<"capture returned "<<capture_success<<endl;
  int decode_success=gray_code_pattern_decode();
  cout<<"decode returned"<<decode_success<<endl;

  return 0;
}