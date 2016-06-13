/**************************************************************************************************
   Software License Agreement (BSD License)

   Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
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
   \file  atlascar_fusion.cpp
   \brief
   \details
   \author David Silva
   \date   June, 2016
 */

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

// To subscribe to image
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Pointcloud2 and PCL conversion
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"


class MultisensorSubs
{
public:

	std::vector<cv::Point3f> lms1CVpoints;
	std::vector<cv::Point3f> lms2CVpoints;
	std::vector<cv::Point3f> ldmrsCVpoints;

	cv::Mat camImage_rect;

	MultisensorSubs(cv::Mat intrinsic_matrix, cv::Mat distortion_coeffs)
	{
		lms1_sub = n.subscribe("/lms151_1/scan", 10, &MultisensorSubs::lms1Callback, this);
		lms2_sub = n.subscribe("/lms151_2/scan", 10, &MultisensorSubs::lms2Callback, this);
		ldmrs_sub = n.subscribe("/ldmrs_1/cloud", 10, &MultisensorSubs::ldmrsCallback, this);
		image_transport::ImageTransport it(n);
		pointgrey_img_sub = it.subscribe("/pointgrey_1/RawImage", 3, &MultisensorSubs::pointgreyCallback, this);
		kinect_sub=n.subscribe("/kinect_3d_1/camera/depth_registered/points", 5, &MultisensorSubs::kinectCallback, this);

		lms151_1_pointCloud_pub = n.advertise<sensor_msgs::PointCloud2>("lms151_1/pointcloud2",10);
		lms151_2_pointCloud_pub = n.advertise<sensor_msgs::PointCloud2>("lms151_2/pointcloud2",10);

		lms1CVpoints.reserve(540); // LMS151 should output 270*2 points (270 degrees; 0.5 angular resolution)
		lms2CVpoints.reserve(540); // LMS151 should output 270*2 points (270 degrees; 0.5 angular resolution)
		ldmrsCVpoints.reserve(680); // MRS400001 should output 4*85*2 points (4 secan planes; 85 degrees; 0.5 angular resolution)

		CM = intrinsic_matrix.clone();
		D = distortion_coeffs.clone();
	}

	~MultisensorSubs(){}

	void lms1Callback (const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		lms1CVpoints.clear();

		sensor_msgs::PointCloud2 ROScloud;
		projector_.projectLaser(*msg, ROScloud);

		lms151_1_pointCloud_pub.publish(ROScloud);

		pcl::PointCloud<pcl::PointXYZ> PCLcloud;
		pcl::fromROSMsg(ROScloud, PCLcloud);

		//std::cout<< "lms1CVpoints ===========================" << std::endl;
		for(int i = 0; i < PCLcloud.points.size(); i++)
		{
			lms1CVpoints.push_back( cv::Point3f(PCLcloud.points[i].x, PCLcloud.points[i].y, PCLcloud.points[i].z) );
			//std::cout<< lms1CVpoints.back() << std::endl;
		}
		//std::cout<< "lms1CVpoints END ===========================" << std::endl;
	}

	void lms2Callback (const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		lms2CVpoints.clear();

		sensor_msgs::PointCloud2 ROScloud;
		projector_.projectLaser(*msg, ROScloud);

		lms151_2_pointCloud_pub.publish(ROScloud);

		pcl::PointCloud<pcl::PointXYZ> PCLcloud;
		pcl::fromROSMsg(ROScloud, PCLcloud);

		for(int i = 0; i < PCLcloud.points.size(); i++)
		{
			lms2CVpoints.push_back( cv::Point3f(PCLcloud.points[i].x, PCLcloud.points[i].y, PCLcloud.points[i].z) );
		}
	}

	void ldmrsCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
	{
		ldmrsCVpoints.clear();
		pcl::PointCloud<pcl::PointXYZ> PCLcloud;
		pcl::fromROSMsg(*msg, PCLcloud);
		//std::cout<< "ldmrsCVpoints ===========================" << std::endl;
		for(int i = 0; i < PCLcloud.points.size(); i++)
		{
			ldmrsCVpoints.push_back( cv::Point3f(PCLcloud.points[i].x, PCLcloud.points[i].y, PCLcloud.points[i].z) );
			//std::cout<< ldmrsCVpoints.back() << std::endl;
		}
		//std::cout<< "ldmrsCVpoints END===========================" << std::endl;
	}

	void pointgreyCallback (const sensor_msgs::ImageConstPtr& msg)
	{
		try
		{
			cv::Mat camImage;
			camImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
			cv::undistort(camImage, camImage_rect, CM, D);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	}

	void kinectCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
	{

	}

private:

	ros::NodeHandle n; // Node handler


	// Initialise subscribers
	ros::Subscriber lms1_sub;
	ros::Subscriber lms2_sub;
	ros::Subscriber ldmrs_sub;
	image_transport::Subscriber pointgrey_img_sub;
	ros::Subscriber kinect_sub;

	laser_geometry::LaserProjection projector_;

	ros::Publisher lms151_1_pointCloud_pub;
	ros::Publisher lms151_2_pointCloud_pub;

	cv::Mat CM;
	cv::Mat D;
};
