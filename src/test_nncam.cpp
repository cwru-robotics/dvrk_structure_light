#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "nncam.h"

HNncam hcam;
void *  image_space = NULL;

static void __stdcall EventCallback(unsigned nEvent, void* pCallbackCtx){
	if (NNCAM_EVENT_IMAGE == nEvent){
		NncamFrameInfoV2 info = { 0 };
		HRESULT hr = Nncam_PullImageV2(hcam, image_space, 24, &info);
		//printf("pull image ok, resolution = %u x %u\n", info.width, info.height);
		
		ros::Time last_update_time = ros::Time::now();
		sensor_msgs::Image i;
		i.header.frame_id = "amscope";
		i.header.stamp.sec = last_update_time.sec;
		i.header.stamp.nsec = last_update_time.nsec;
		fillImage(i, sensor_msgs::image_encodings::RGB8, info.height, info.width, 3 * info.width, reinterpret_cast<const void*>(image_space));


    cv::Mat image;

		cv_bridge::CvImage img_bridge;

		// img_bridge=cv_bridge::CvImage(,,);
  	cv_bridge::toCvCopy( i ,  sensor_msgs::image_encodings::BGR8)->image;
		std::string sensor_img_status= (i.data.empty())?"empty":"populated";
		std::string cv_img_status= (image.empty())?"empty":"populated";

		ROS_WARN_STREAM("sensor image "<<sensor_img_status<<". CV image "<<cv_img_status);
		cv::imshow("view", image);
		// pub->publish(i);
		ros::Duration(0.1).sleep();

  }
}

int main(int argc, char** argv){

	ros::init(argc, argv, "amscope_driver");
	ros::NodeHandle nh;
	
	hcam = Nncam_Open(NULL);
	if (NULL == hcam){
        	printf("No cameras found. Exiting.\n");
		return 0;
	}
	
	NncamDeviceV2 camera_array[NNCAM_MAX];
	unsigned camera_cnt = Nncam_EnumV2(camera_array);
	
	printf("Detected %u cameras.\n", camera_cnt);
	//publishers = std::vector<ros::Publisher>(camera_cnt);
	
	int w, h;
	HRESULT hr = Nncam_get_Size(hcam, &w, &h);
	
	
	image_space = malloc(TDIBWIDTHBYTES(24 * w) * h);
	hr = Nncam_StartPullModeWithCallback(hcam, EventCallback, NULL);


  cv::namedWindow("view");
  cv::namedWindow("view",cv::WINDOW_AUTOSIZE);

  ROS_INFO("created view  window");



  

 	ros::spin();

  cv::destroyWindow("view");
    
	Nncam_Close(hcam);
    
	/* cleanup */
	if (image_space){
		free(image_space);
	}
	return 0;
}