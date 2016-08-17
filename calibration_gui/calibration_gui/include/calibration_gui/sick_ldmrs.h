/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
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
\file  sick_ldmrs.h
\brief Global include file
\author Marcelo Pereira
\date   December, 2015
*/

#ifndef _SICK_LDMRS_H_
#define _SICK_LDMRS_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <rosbag/bag.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <colormap/colormap.h>
#include <utility>
#include <map>
#include <fstream>
#include <string>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include "lidar_segmentation/lidar_segmentation.h"

using namespace std;

//Shared pointer to the Point class
typedef boost::shared_ptr<Point> PointPtr;
typedef boost::shared_ptr< ::sensor_msgs::LaserScan> LaserScanPtr;

/**
  \class MultiScan
  \brief Class to handle the several scans from the sick ld-mrs laser
  \author Marcelo Pereira
 */
class MultiScan
{
public:
    vector<PointPtr> Points;
};
typedef boost::shared_ptr<MultiScan> MultiScanPtr;

/**
  \class sickLMSscan
  \brief Class to subscribe the scans from the sick ld-mrs laser
  \author David Silva
 */
class sickLDMRSscan
{
public:
    ros::NodeHandle n_;
    ros::Subscriber scan0_subscriber;
    ros::Subscriber scan1_subscriber;
    ros::Subscriber scan2_subscriber;
    ros::Subscriber scan3_subscriber;
    sensor_msgs::LaserScan scan0;
    sensor_msgs::LaserScan scan1;
    sensor_msgs::LaserScan scan2;
    sensor_msgs::LaserScan scan3;

/**
	@brief Constructor. Subscription of the point cloud from the SICK LD-MRS laser sensor
	@param nodeToSub node name to subscribe
*/
    sickLDMRSscan(const string &nodeToSub)
    {
        //Topics I want to subscribe
        scan0_subscriber=n_.subscribe("/" + nodeToSub + "/scan0", 1000, &sickLDMRSscan::scan0Update, this);
        scan1_subscriber=n_.subscribe("/" + nodeToSub + "/scan1", 1000, &sickLDMRSscan::scan1Update, this);
        scan2_subscriber=n_.subscribe("/" + nodeToSub + "/scan2", 1000, &sickLDMRSscan::scan2Update, this);
        scan3_subscriber=n_.subscribe("/" + nodeToSub + "/scan3", 1000, &sickLDMRSscan::scan3Update, this);
    }

/**
   @brief Callback function that is called when a message arrives to the topic: "/" + nodeToSub + "/scan0"
   @param msg message received from the SICK LD-MRS laser sensor
   @return void
*/
    void scan0Update(const sensor_msgs::LaserScan& msg)
    {
        scan0=msg;
        //ROS_INFO("Scan time: %lf ", msg.ranges[1]);
    }

/**
   @brief Callback function that is called when a message arrives to the topic: "/" + nodeToSub + "/scan1"
   @param msg message received from the SICK LD-MRS laser sensor
   @return void
*/
    void scan1Update(const sensor_msgs::LaserScan& msg)
    {
        scan1=msg;
        //ROS_INFO("Scan time: %lf ", msg.ranges[1]);
    }
/**
   @brief Callback function that is called when a message arrives to the topic: "/" + nodeToSub + "/scan2"
   @param msg message received from the SICK LD-MRS laser sensor
   @return void
*/
    void scan2Update(const sensor_msgs::LaserScan& msg)
    {
        scan2=msg;
        //ROS_INFO("Scan time: %lf ", msg.ranges[1]);
    }

/**
   @brief Callback function that is called when a message arrives to the topic: "/" + nodeToSub + "/scan3"
   @param msg message received from the SICK LD-MRS laser sensor
   @return void
*/
    void scan3Update(const sensor_msgs::LaserScan& msg)
    {
        scan3=msg;
        //ROS_INFO("Scan time: %lf ", msg.ranges[1]);
    }
};

double find_circle(vector<ClusterPtr> clusters, vector<ClusterPtr>& circleP, int layer);
void calculateSphereCentroid(vector<geometry_msgs::Point> center, geometry_msgs::PointStamped &sphereCentroid, vector<double> radius);
void rotatePoints(double& x,double& y, double& z, double angle);
void convertDataToXYZ(sensor_msgs::LaserScan scan, vector<C_DataFromFilePtr>& data_gt, double rot);
#endif
