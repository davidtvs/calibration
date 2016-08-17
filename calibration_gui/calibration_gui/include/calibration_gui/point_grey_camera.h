/**************************************************************************************************
   Software License Agreement (BSD License)

   Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification, are permitted
   provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
   FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
   IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************************************/
/**
   \file  point_grey_camera.h
   \brief Header file for point_grey_camera.cpp. Ball detection and computation of its relevant proprieties with a Point Grey camera.
   \author David Silva
   \date   July, 2016
 */

#ifndef _POINT_GREY_CAMERA_H_
#define _POINT_GREY_CAMERA_H_

#include <stdio.h>
#include <string>
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <visualization_msgs/MarkerArray.h>
#include "visualization_rviz_swissranger.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>

double BALL_DIAMETER;

using namespace cv;
using namespace std;

/**
   \class CameraRaw
   \brief Class to subscribe and acquire the images from the Point Grey camera
   \author David Silva
 */
class CameraRaw
{
public:
	ros::NodeHandle n_;
	image_transport::Subscriber subs_cam_image;
	Mat camImage;

/**
	@brief Constructor. Subscription to the topic that contains the images acquired from the Point Grey camera.
	@param nodeToSub node name to subscribe
*/
	CameraRaw(const string &nodeToSub)
	{
		image_transport::ImageTransport it(n_);
		subs_cam_image = it.subscribe ("/" + nodeToSub + "/RawImage", 1, &CameraRaw::imageUpdate, this);
	}

/**
   @brief Callback function that is called when a message arrives to the topic: "/" + nodeToSub + "/RawImage"
   @param msg message received from the SICK LD-MRS laser sensor
   @return void
*/
	void imageUpdate(const sensor_msgs::ImageConstPtr& msg)
	{
		try
		{
			camImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
		//ROS_INFO("Scan time: %lf ", msg.ranges[1]);
	}
};

void CreateTrackbarsAndWindows ();

void ImageProcessing(Mat &img);

void HoughDetection(const Mat &img, const Mat& imgBinary );

void PolygonalCurveDetection( Mat &img, Mat &imgBinary );

void CentroidPub( const pcl::PointXYZ centroid, const pcl::PointXYZ centroidRadius );

void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

#endif
