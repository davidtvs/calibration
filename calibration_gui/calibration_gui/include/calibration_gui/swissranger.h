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
\file  swissranger.h
\brief Global include file
\author Marcelo Pereira, David Silva
\date   Maio, 2016
*/
#ifndef _SWISSRANGER_H_
#define _SWISSRANGER_H_

#include <eigen3/Eigen/Dense>

double BALL_DIAMETER;

using namespace std;

/**
  \class swissranger
  \brief Class to handle the point cloud from the swissranger
  \author David Silva
 */
class swissranger
{
public:
    ros::NodeHandle n_;
    ros::Subscriber pointCloud_subscriber;
    sensor_msgs::PointCloud cloud; /**< point cloud from the swissranger. */

/**
	@brief Constructor. Subscription of the point cloud from the swissranger
	@param nodeToSub node name to subscribe
*/
    swissranger(const string &nodeToSub)
    {
        //Topics I want to subscribe
        pointCloud_subscriber=n_.subscribe("/" + nodeToSub + "/pointcloud_raw", 1, &swissranger::pointCloudUpdate, this);
    }

/**
@brief swissranger class destructor
*/
    ~swissranger(){}

/**
   @brief Callback function that is called when a message arrives
   @param msg message received from the Swissranger sensor
   @return void
 */
    void pointCloudUpdate(const sensor_msgs::PointCloud & msg)
    {
        cloud=msg;
        //ROS_INFO("Scan time: %lf ", msg.data[0]);
    }
};

void writeFile(Eigen::VectorXf sphereCoeffsRefined);
#endif
