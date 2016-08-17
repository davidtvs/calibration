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
   \file  swissranger.h
   \brief Header file for kinect.cpp. Ball detection and computation of its relevant proprieties with the Kinect 3D-depth sensor.
   \author David Silva
   \date   July, 2016
 */
 

#ifndef _KINECT_H_
#define _KINECT_H_

#include "ros/ros.h"
#include <cmath>
#include <algorithm>
#include <geometry_msgs/Point32.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/io/pcd_io.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>


#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Dense>

#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

double BALL_DIAMETER;

using namespace std;

/**
   \class kinect
   \brief Class to handle the point cloud from the Kinect 3D-depth sensor
   \author David Silva
 */
class kinect
{
public:
	ros::NodeHandle n_;
	ros::Subscriber pointCloud_subscriber;
	pcl::PointCloud<pcl::PointXYZ> cloud;

/**
   @brief Constructor. Subscribes to the point cloud from the Kinect 3D-depth sensor.
   @param nodeToSub node name to subscribe 
 */
	kinect(const string &nodeToSub)
	{
		//Topics I want to subscribe
		pointCloud_subscriber=n_.subscribe("/" + nodeToSub + "/camera/depth_registered/points",
		                                   1, &kinect::pointCloudUpdate, this);
		cout << "/" << nodeToSub << "/camera/depth_registered/points" << endl;
	}
	
/**
	@brief Class destructor
*/
	~kinect(){}

/**
   @brief Callback function that is called when a message arrives to the topic: "/" + nodeToSub + "/camera/depth_registered/points"
   @param msg message received from the Kinect 3D-depth sensor
   @return void
*/
	void pointCloudUpdate(const sensor_msgs::PointCloud2ConstPtr & msg)
	{
		pcl::fromROSMsg(*msg, cloud);
		//ROS_INFO("Scan time: %lf ", msg.data[0]);
	}
};

void writeFile(Eigen::VectorXf sphereCoeffsRefined);
#endif
