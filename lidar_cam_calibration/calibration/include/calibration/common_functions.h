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
\file  common_functions.h
\brief Global include file
\author Marcelo Pereira
\date   December, 2015
*/

#ifndef _common_functions_H_
#define _common_functions_H_

#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <string>
#include "lidar_segmentation/lidar_segmentation.h"

#define BALL_DIAMETER 0.99

using namespace std;


/**
  \class sickLMSscan
  \brief Class to subscribe the scans from the two sick lms151 lasers
  \author Marcelo Pereira
 */
class sickLMSscan
{
public:
    ros::NodeHandle n_;
    ros::Subscriber laser1_subscriber;
    ros::Subscriber laser2_subscriber;
    sensor_msgs::LaserScan scanLaser1;
    sensor_msgs::LaserScan scanLaser2;

    sickLMSscan()
    {
        //Topics I want to subscribe
        laser1_subscriber=n_.subscribe("/laser_1/scan", 1000, &sickLMSscan::laser1Update, this);
        laser2_subscriber=n_.subscribe("/laser_2/scan", 1000, &sickLMSscan::laser2Update, this);
    }

    void laser1Update(const sensor_msgs::LaserScan& msg)
    {
        scanLaser1=msg;
        //ROS_INFO("Scan time: %lf ", msg.ranges[1]);
    }

    void laser2Update(const sensor_msgs::LaserScan& msg)
    {
        scanLaser2=msg;
        //ROS_INFO("Scan time: %lf ", msg.ranges[1]);
    }
};

class LidarClusters
{
public:
    vector<ClusterPtr> Clusters;
};
typedef boost::shared_ptr<LidarClusters> LidarClustersPtr;

void circlePoints(vector<ClusterPtr>& circle_points, double radius, double centre[3], int number_points);
void CalculateCircle(ClusterPtr cluster, double& R, Point& Center);
void convertDataToXY(sensor_msgs::LaserScan scan, C_DataFromFilePtr& data_gt);
#endif
