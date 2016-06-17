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
 \file  visualization_rviz_lms.cpp
 \brief Illustration of the ball detection on the laser data
 \details It is represented the segmentation of the laser data, and the ball when detected
 \author Marcelo Pereira
 \date   December, 2015
*/

#include <lidar_segmentation/lidar_segmentation.h>
#include <lidar_segmentation/clustering.h>
#include "calibration_gui/visualization_rviz_lms.h"
#include <visualization_msgs/Marker.h>

/**
@brief Markers publication for the visualization of the laser scan, circle detected and ball detected
@param[in] clusters_nn clusters from the laser scan
@param[in] circlePoints points for the representation of the detected circles
@param[in] sphere center coordinates of the ball
@return vector<visualization_msgs::Marker>
*/
vector<visualization_msgs::Marker> createTargetMarkers(vector<ClusterPtr>& clusters_nn, vector<ClusterPtr>& circlePoints, Point sphere )
{

    static Markers marker_list;

    //Reduce the elements status, ADD to REMOVE and REMOVE to delete
    marker_list.decrement();

    // Create a colormap
    class_colormap colormap("hsv",10, 1, false);

    visualization_msgs::Marker marker_ids_scan;
    visualization_msgs::Marker marker_ids_circle;
    visualization_msgs::Marker marker_ids_sphere;

    visualization_msgs::Marker marker_clusters_scan;
    visualization_msgs::Marker marker_clusters_circle;
    visualization_msgs::Marker marker_sphere;

    marker_ids_scan.header.frame_id = "/my_frame";
    marker_ids_scan.header.stamp = ros::Time::now();
    marker_ids_circle.header.frame_id = "/my_frame";
    marker_ids_circle.header.stamp = ros::Time::now();
    marker_ids_sphere.header.frame_id = "/my_frame";
    marker_ids_sphere.header.stamp = ros::Time::now();

    marker_clusters_scan.header.frame_id = "/my_frame";
    marker_clusters_scan.header.stamp = marker_ids_scan.header.stamp;
    marker_clusters_circle.header.frame_id = "/my_frame";
    marker_clusters_circle.header.stamp = marker_ids_circle.header.stamp;
    marker_sphere.header.frame_id = "/my_frame";
    marker_sphere.header.stamp = marker_sphere.header.stamp;

    marker_ids_scan.ns = "ids_scan";
    marker_ids_scan.action = visualization_msgs::Marker::ADD;
    marker_ids_circle.ns = "ids_circle";
    marker_ids_circle.action = visualization_msgs::Marker::ADD;
    marker_ids_sphere.ns = "ids_sphere";
    marker_ids_sphere.action = visualization_msgs::Marker::ADD;

    marker_clusters_scan.ns = "clusters_scan";
    marker_clusters_scan.action = visualization_msgs::Marker::ADD;
    marker_clusters_circle.ns = "clusters_circle";
    marker_clusters_circle.action = visualization_msgs::Marker::ADD;
    marker_sphere.ns = "sphere";
    marker_sphere.action = visualization_msgs::Marker::ADD;

    marker_ids_scan.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ids_circle.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ids_sphere.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker_clusters_scan.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_clusters_circle.type = visualization_msgs::Marker::LINE_STRIP;
    marker_sphere.type = visualization_msgs::Marker::SPHERE_LIST;

    marker_ids_scan.scale.x = 0.1;
    marker_ids_scan.scale.y = 0.1;
    marker_ids_scan.scale.z = 0.1;
    marker_ids_circle.scale.x = 0.1;
    marker_ids_circle.scale.y = 0.1;
    marker_ids_circle.scale.z = 0.1;
    marker_ids_sphere.scale.x = 0.1;
    marker_ids_sphere.scale.y = 0.1;
    marker_ids_sphere.scale.z = 0.1;

    marker_clusters_scan.scale.x = 0.05;
    marker_clusters_scan.scale.y = 0.05;
    marker_clusters_scan.scale.z = 0.05;
    marker_clusters_circle.scale.x = 0.03;
    marker_sphere.scale.x = 1.07;
    marker_sphere.scale.y = 1.07;
    marker_sphere.scale.z = 1.07;

    marker_ids_scan.color.a = 1.0;
    marker_ids_scan.color.r = 1.0;
    marker_ids_scan.color.g = 1.0;
    marker_ids_scan.color.b = 1.0;
    marker_ids_circle.color.a = 1.0;
    marker_ids_circle.color.r = 1.0;
    marker_ids_circle.color.g = 1.0;
    marker_ids_circle.color.b = 1.0;
    marker_ids_sphere.color.a = 1.0;
    marker_ids_sphere.color.r = 1.0;
    marker_ids_sphere.color.g = 1.0;
    marker_ids_sphere.color.b = 1.0;


    for ( uint i = 0; i<  clusters_nn.size() ; i++)
    {
        ClusterPtr cluster_nn = clusters_nn[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_nn->support_points.size();h++)
        {
            geometry_msgs::Point pt;
            pt.x=cluster_nn->support_points[h]->x;
            pt.y=cluster_nn->support_points[h]->y;
            pt.z= cluster_nn->support_points[h]->z;

            marker_clusters_scan.points.push_back(pt);
            marker_clusters_scan.colors.push_back(color);
        }

        marker_ids_scan.pose.position.x = cluster_nn->centroid->x;
        marker_ids_scan.pose.position.y = cluster_nn->centroid->y;
        marker_ids_scan.pose.position.z = 1.3;

        //text
        boost::format fm("%d");
        fm % cluster_nn->id;

        marker_ids_scan.text = fm.str();
        marker_ids_scan.id = cluster_nn->id;

        marker_list.update(marker_ids_scan);
        marker_list.update(marker_clusters_scan);

    } //end for

    //circle fit

    for ( uint i = 0; i< circlePoints.size() ; i++)
    {
        ClusterPtr cluster_circle = circlePoints[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_circle->support_points.size();h++)
        {
            geometry_msgs::Point pt;
            pt.x=cluster_circle->support_points[h]->x;
            pt.y=cluster_circle->support_points[h]->y;
            pt.z= 0;

            marker_clusters_circle.points.push_back(pt);
            marker_clusters_circle.colors.push_back(color);
        }

        marker_ids_circle.pose.position.x = cluster_circle->centroid->x;
        marker_ids_circle.pose.position.y = cluster_circle->centroid->y;
        marker_ids_circle.pose.position.z = 1.5;

        //text
        boost::format fm("%d");
        fm % 1.0;

        marker_ids_circle.text = fm.str();
        marker_ids_circle.id = 1.0;

        marker_list.update(marker_ids_circle);
        marker_list.update(marker_clusters_circle);

    } //end for

    //sphere

    if(sphere.x!=0)
    {
        std_msgs::ColorRGBA color = colormap.color(0);

        geometry_msgs::Point pt;
        pt.x=sphere.x;
        pt.y= sphere.y;
        pt.z= sphere.z;
        // cout<<"x "<<pt.x<<endl;
        // cout<<"y "<<pt.y<<endl;
        // cout<<"z "<<pt.z<<endl;
        marker_sphere.points.push_back(pt);
        marker_sphere.colors.push_back(color);

        marker_ids_sphere.pose.position.x = 0;
        marker_ids_sphere.pose.position.y = 0;
        marker_ids_sphere.pose.position.z = 0;

        //text
        boost::format fm("%d");
        fm % 1.0;

        marker_ids_sphere.text = fm.str();
        marker_ids_sphere.id = 1.0;

        marker_list.update(marker_ids_sphere);
        marker_list.update(marker_sphere);
    }

    //Remove markers that should not be transmitted
    marker_list.clean();

    //Clean the marker_vector and put new markers in it;
    return marker_list.getOutgoingMarkers();

} //end function
