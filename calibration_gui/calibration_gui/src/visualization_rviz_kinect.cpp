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
 \file  visualization_rviz_kinect.cpp
 \brief Illustration of the ball detection on the Kinect 3D-depth sensor data. Similar to visualization_rviz_swissranger.cpp.
 \author  David Silva
 \date   July, 2016
*/

#include <lidar_segmentation/lidar_segmentation.h>
#include <lidar_segmentation/clustering.h>
#include "calibration_gui/visualization_rviz_swissranger.h"
#include <vector>

/**
@brief Markers publication for the visualization of the ball detected
@param[in] sphereCenter center coordinates of the ball
@return vector<visualization_msgs::Marker>
*/
vector<visualization_msgs::Marker> createTargetMarkers(pcl::PointXYZ sphereCenter )
{

    static Markers marker_list;

    //Reduce the elements status, ADD to REMOVE and REMOVE to delete
    marker_list.decrement();

    visualization_msgs::Marker marker_sphere;

    marker_sphere.header.frame_id = "/my_frame";
    marker_sphere.header.stamp = ros::Time::now();

    marker_sphere.ns = "sphere";
    marker_sphere.action = visualization_msgs::Marker::ADD;

    marker_sphere.type = visualization_msgs::Marker::SPHERE_LIST;

    marker_sphere.scale.x = 0.9;
    marker_sphere.scale.y = 0.9;
    marker_sphere.scale.z = 0.9;


    //sphere

    if(sphereCenter.x!=0)
    {
        geometry_msgs::Point pt;
        pt.x=sphereCenter.x;
        pt.y=sphereCenter.y;
        pt.z=sphereCenter.z;
        marker_sphere.points.push_back(pt);
        marker_sphere.color.a=1;
        marker_sphere.color.g=1;

        marker_list.update(marker_sphere);
    }

    //Remove markers that should not be transmitted
    marker_list.clean();

    //Clean the marker_vector and put new markers in it;
    return marker_list.getOutgoingMarkers();

} //end function
