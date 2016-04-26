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
 \file  visualization_rviz_ldmrs.cpp
 \brief Illustration of the ball detection on the laser data
 \details It is represented the segmentation of the laser data, and the ball when detected
 \author Marcelo Pereira
 \date   December, 2015
*/

#include <lidar_segmentation/lidar_segmentation.h>
#include <lidar_segmentation/clustering.h>
#include "calibration/common_functions.h"
#include "calibration/visualization_rviz_ldmrs.h"
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <iterator>
#include <iostream>

/**
@brief Markers publication for the visualization of the laser scans, circle detected and ball detected
@param[in] sickLidarClusters_nn segmentatio of the several laser scans
@param[in] circlePoints points for the representation of the detected circles
@param[in] sphere center coordinates of the ball
@param[in] radius radius of the circles detected
@return vector<visualization_msgs::Marker>
*/
vector<visualization_msgs::Marker> createTargetMarkers(vector<LidarClustersPtr>& sickLidarClusters_nn, vector<LidarClustersPtr>& circlePoints, Point sphere, vector<double> radius )
{
    static Markers marker_list;

    //Reduce the elements status, ADD to REMOVE and REMOVE to delete
    marker_list.decrement();

    // Create a colormap
    class_colormap colormap("hsv",10, 1, false);

    visualization_msgs::Marker marker_ids_scan0;
    visualization_msgs::Marker marker_ids_scan1;
    visualization_msgs::Marker marker_ids_scan2;
    visualization_msgs::Marker marker_ids_scan3;
    visualization_msgs::Marker marker_ids_circle0;
    visualization_msgs::Marker marker_ids_circle1;
    visualization_msgs::Marker marker_ids_circle2;
    visualization_msgs::Marker marker_ids_circle3;
    visualization_msgs::Marker marker_ids_sphere;

    visualization_msgs::Marker marker_clusters_scan0;
    visualization_msgs::Marker marker_clusters_scan1;
    visualization_msgs::Marker marker_clusters_scan2;
    visualization_msgs::Marker marker_clusters_scan3;
    visualization_msgs::Marker marker_clusters_circle0;
    visualization_msgs::Marker marker_clusters_circle1;
    visualization_msgs::Marker marker_clusters_circle2;
    visualization_msgs::Marker marker_clusters_circle3;
    visualization_msgs::Marker marker_sphere;

    marker_ids_scan0.header.frame_id = "/my_frame";
    marker_ids_scan0.header.stamp = ros::Time::now();
    marker_ids_scan1.header.frame_id = "/my_frame";
    marker_ids_scan1.header.stamp = ros::Time::now();
    marker_ids_scan2.header.frame_id = "/my_frame";
    marker_ids_scan2.header.stamp = ros::Time::now();
    marker_ids_scan3.header.frame_id = "/my_frame";
    marker_ids_scan3.header.stamp = ros::Time::now();
    marker_ids_circle0.header.frame_id = "/my_frame";
    marker_ids_circle0.header.stamp = ros::Time::now();
    marker_ids_circle1.header.frame_id = "/my_frame";
    marker_ids_circle1.header.stamp = ros::Time::now();
    marker_ids_circle2.header.frame_id = "/my_frame";
    marker_ids_circle2.header.stamp = ros::Time::now();
    marker_ids_circle3.header.frame_id = "/my_frame";
    marker_ids_circle3.header.stamp = ros::Time::now();
    marker_ids_sphere.header.frame_id = "/my_frame";
    marker_ids_sphere.header.stamp = ros::Time::now();

    marker_clusters_scan0.header.frame_id = "/my_frame";
    marker_clusters_scan0.header.stamp = marker_ids_scan0.header.stamp;
    marker_clusters_scan1.header.frame_id = "/my_frame";
    marker_clusters_scan1.header.stamp = marker_ids_scan1.header.stamp;
    marker_clusters_scan2.header.frame_id = "/my_frame";
    marker_clusters_scan2.header.stamp = marker_ids_scan2.header.stamp;
    marker_clusters_scan3.header.frame_id = "/my_frame";
    marker_clusters_scan3.header.stamp = marker_ids_scan3.header.stamp;
    marker_clusters_circle0.header.frame_id = "/my_frame";
    marker_clusters_circle0.header.stamp = marker_ids_circle0.header.stamp;
    marker_clusters_circle1.header.frame_id = "/my_frame";
    marker_clusters_circle1.header.stamp = marker_ids_circle1.header.stamp;
    marker_clusters_circle2.header.frame_id = "/my_frame";
    marker_clusters_circle2.header.stamp = marker_ids_circle2.header.stamp;
    marker_clusters_circle3.header.frame_id = "/my_frame";
    marker_clusters_circle3.header.stamp = marker_ids_circle3.header.stamp;
    marker_sphere.header.frame_id = "/my_frame";
    marker_sphere.header.stamp = marker_sphere.header.stamp;

    marker_ids_scan0.ns = "ids_scan0";
    marker_ids_scan0.action = visualization_msgs::Marker::ADD;
    marker_ids_scan1.ns = "ids_scan1";
    marker_ids_scan1.action = visualization_msgs::Marker::ADD;
    marker_ids_scan2.ns = "ids_scan2";
    marker_ids_scan2.action = visualization_msgs::Marker::ADD;
    marker_ids_scan3.ns = "ids_scan3";
    marker_ids_scan3.action = visualization_msgs::Marker::ADD;
    marker_ids_circle0.ns = "ids_circle0";
    marker_ids_circle0.action = visualization_msgs::Marker::ADD;
    marker_ids_circle1.ns = "ids_circle1";
    marker_ids_circle1.action = visualization_msgs::Marker::ADD;
    marker_ids_circle2.ns = "ids_circle2";
    marker_ids_circle2.action = visualization_msgs::Marker::ADD;
    marker_ids_circle3.ns = "ids_circle3";
    marker_ids_circle3.action = visualization_msgs::Marker::ADD;
    marker_ids_sphere.ns = "ids_sphere";
    marker_ids_sphere.action = visualization_msgs::Marker::ADD;

    marker_clusters_scan0.ns = "clusters_scan0";
    marker_clusters_scan0.action = visualization_msgs::Marker::ADD;
    marker_clusters_scan1.ns = "clusters_scan1";
    marker_clusters_scan1.action = visualization_msgs::Marker::ADD;
    marker_clusters_scan2.ns = "clusters_scan2";
    marker_clusters_scan2.action = visualization_msgs::Marker::ADD;
    marker_clusters_scan3.ns = "clusters_scan3";
    marker_clusters_scan3.action = visualization_msgs::Marker::ADD;
    marker_clusters_circle0.ns = "clusters_circle0";
    marker_clusters_circle0.action = visualization_msgs::Marker::ADD;
    marker_clusters_circle1.ns = "clusters_circle1";
    marker_clusters_circle1.action = visualization_msgs::Marker::ADD;
    marker_clusters_circle2.ns = "clusters_circle2";
    marker_clusters_circle2.action = visualization_msgs::Marker::ADD;
    marker_clusters_circle3.ns = "clusters_circle3";
    marker_clusters_circle3.action = visualization_msgs::Marker::ADD;
    marker_sphere.ns = "sphere";
    marker_sphere.action = visualization_msgs::Marker::ADD;

    marker_ids_scan0.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ids_scan1.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ids_scan2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ids_scan3.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ids_circle0.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ids_circle1.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ids_circle2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ids_circle3.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ids_sphere.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker_clusters_scan0.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_clusters_scan1.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_clusters_scan2.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_clusters_scan3.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_clusters_circle0.type = visualization_msgs::Marker::LINE_STRIP;
    marker_clusters_circle1.type = visualization_msgs::Marker::LINE_STRIP;
    marker_clusters_circle2.type = visualization_msgs::Marker::LINE_STRIP;
    marker_clusters_circle3.type = visualization_msgs::Marker::LINE_STRIP;
    marker_sphere.type = visualization_msgs::Marker::SPHERE_LIST;

    marker_ids_scan0.scale.x = 0.1;
    marker_ids_scan0.scale.y = 0.1;
    marker_ids_scan0.scale.z = 0.1;
    marker_ids_scan1.scale.x = 0.1;
    marker_ids_scan1.scale.y = 0.1;
    marker_ids_scan1.scale.z = 0.1;
    marker_ids_scan2.scale.x = 0.1;
    marker_ids_scan2.scale.y = 0.1;
    marker_ids_scan2.scale.z = 0.1;
    marker_ids_scan3.scale.x = 0.1;
    marker_ids_scan3.scale.y = 0.1;
    marker_ids_scan3.scale.z = 0.1;
    marker_ids_circle0.scale.x = 0.1;
    marker_ids_circle0.scale.y = 0.1;
    marker_ids_circle0.scale.z = 0.1;
    marker_ids_circle1.scale.x = 0.1;
    marker_ids_circle1.scale.y = 0.1;
    marker_ids_circle1.scale.z = 0.1;
    marker_ids_circle2.scale.x = 0.1;
    marker_ids_circle2.scale.y = 0.1;
    marker_ids_circle2.scale.z = 0.1;
    marker_ids_circle3.scale.x = 0.1;
    marker_ids_circle3.scale.y = 0.1;
    marker_ids_circle3.scale.z = 0.1;
    marker_ids_sphere.scale.x = 0.1;
    marker_ids_sphere.scale.y = 0.1;
    marker_ids_sphere.scale.z = 0.1;

    marker_clusters_scan0.scale.x = 0.08;
    marker_clusters_scan0.scale.y = 0.08;
    marker_clusters_scan0.scale.z = 0.08;
    marker_clusters_scan1.scale.x = 0.08;
    marker_clusters_scan1.scale.y = 0.08;
    marker_clusters_scan1.scale.z = 0.08;
    marker_clusters_scan2.scale.x = 0.08;
    marker_clusters_scan2.scale.y = 0.08;
    marker_clusters_scan2.scale.z = 0.08;
    marker_clusters_scan3.scale.x = 0.08;
    marker_clusters_scan3.scale.y = 0.08;
    marker_clusters_scan3.scale.z = 0.08;
    marker_clusters_circle0.scale.x = 0.03;
    marker_clusters_circle1.scale.x = 0.03;
    marker_clusters_circle2.scale.x = 0.03;
    marker_clusters_circle3.scale.x = 0.03;
    marker_sphere.scale.x = 1.07;
    marker_sphere.scale.y = 1.07;
    marker_sphere.scale.z = 1.07;

    marker_ids_scan0.color.a = 1.0;
    marker_ids_scan0.color.r = 1.0;
    marker_ids_scan0.color.g = 1.0;
    marker_ids_scan0.color.b = 1.0;
    marker_ids_scan1.color.a = 1.0;
    marker_ids_scan1.color.r = 1.0;
    marker_ids_scan1.color.g = 1.0;
    marker_ids_scan1.color.b = 1.0;
    marker_ids_scan2.color.a = 1.0;
    marker_ids_scan2.color.r = 1.0;
    marker_ids_scan2.color.g = 1.0;
    marker_ids_scan2.color.b = 1.0;
    marker_ids_scan3.color.a = 1.0;
    marker_ids_scan3.color.r = 1.0;
    marker_ids_scan3.color.g = 1.0;
    marker_ids_scan3.color.b = 1.0;
    marker_ids_circle0.color.a = 1.0;
    marker_ids_circle0.color.r = 1.0;
    marker_ids_circle0.color.g = 1.0;
    marker_ids_circle0.color.b = 1.0;
    marker_ids_circle1.color.a = 1.0;
    marker_ids_circle1.color.r = 1.0;
    marker_ids_circle1.color.g = 1.0;
    marker_ids_circle1.color.b = 1.0;
    marker_ids_circle2.color.a = 1.0;
    marker_ids_circle2.color.r = 1.0;
    marker_ids_circle2.color.g = 1.0;
    marker_ids_circle2.color.b = 1.0;
    marker_ids_circle3.color.a = 1.0;
    marker_ids_circle3.color.r = 1.0;
    marker_ids_circle3.color.g = 1.0;
    marker_ids_circle3.color.b = 1.0;
    marker_ids_sphere.color.a = 1.0;
    marker_ids_sphere.color.r = 1.0;
    marker_ids_sphere.color.g = 1.0;
    marker_ids_sphere.color.b = 1.0;

    vector<double>::iterator max = max_element(radius.begin(),radius.end());
    int id1 = distance(radius.begin(),max);
    radius[id1]=0;
    max = max_element(radius.begin(),radius.end());
    int id2 = distance(radius.begin(),max);
    radius[id2]=0;
    max = max_element(radius.begin(),radius.end());
    int id3 = distance(radius.begin(),max);
    radius[id3]=0;
    max = max_element(radius.begin(),radius.end());
    int id4 = distance(radius.begin(),max);

    //green
    if(id1==0)
    {
        marker_clusters_circle0.color.g=1.0f;
        marker_clusters_circle0.color.a=1.0;
    }
    else if(id1==1)
    {
        marker_clusters_circle1.color.g=1.0f;
        marker_clusters_circle1.color.a=1.0;
    }
    else if(id1==2)
    {
        marker_clusters_circle2.color.g=1.0f;
        marker_clusters_circle2.color.a=1.0;
    }
    else if(id1==3)
    {
        marker_clusters_circle3.color.g=1.0f;
        marker_clusters_circle3.color.a=1.0;
    }

    //blue
    if(id2==0)
    {
        marker_clusters_circle0.color.b=1.0;
        marker_clusters_circle0.color.a=1.0;
    }
    else if(id2==1)
    {
        marker_clusters_circle1.color.b=1.0;
        marker_clusters_circle1.color.a=1.0;
    }
    else if(id2==2)
    {
        marker_clusters_circle2.color.b=1.0;
        marker_clusters_circle2.color.a=1.0;
    }
    else if(id2==3)
    {
        marker_clusters_circle3.color.b=1.0;
        marker_clusters_circle3.color.a=1.0;
    }

    //yellow
    if(id3==0)
    {
        marker_clusters_circle0.color.r=1.0;
        marker_clusters_circle0.color.g=1.0;
        marker_clusters_circle0.color.a=1.0;
    }
    else if(id3==1)
    {
        marker_clusters_circle1.color.r=1.0;
        marker_clusters_circle1.color.g=1.0;
        marker_clusters_circle1.color.a=1.0;
    }
    else if(id3==2)
    {
        marker_clusters_circle2.color.r=1.0;
        marker_clusters_circle2.color.g=1.0;
        marker_clusters_circle2.color.a=1.0;
    }
    else if(id3==3)
    {
        marker_clusters_circle3.color.r=1.0;
        marker_clusters_circle3.color.g=1.0;
        marker_clusters_circle3.color.a=1.0;
    }

    //red
    if(id4==0)
    {
        marker_clusters_circle0.color.r=1.0;
        marker_clusters_circle0.color.a=1.0;
    }
    else if(id4==1)
    {
        marker_clusters_circle1.color.r=1.0;
        marker_clusters_circle1.color.a=1.0;
    }
    else if(id4==2)
    {
        marker_clusters_circle2.color.r=1.0;
        marker_clusters_circle2.color.a=1.0;
    }
    else if(id4==3)
    {
        marker_clusters_circle3.color.r=1.0;
        marker_clusters_circle3.color.a=1.0;
    }

    //Cluster from scan0
    vector<ClusterPtr> clusters_scan0 = sickLidarClusters_nn[0]->Clusters;

    for ( uint i = 0; i<  clusters_scan0.size() ; i++)
    {
        ClusterPtr cluster_nn = clusters_scan0[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_nn->support_points.size();h++)
        {
            geometry_msgs::Point pt;
            pt.x=cluster_nn->support_points[h]->x;
            pt.y=cluster_nn->support_points[h]->y;
            pt.z=cluster_nn->support_points[h]->z;
            //cout<<"z "<<pt.z<<endl;

            marker_clusters_scan0.points.push_back(pt);
            marker_clusters_scan0.colors.push_back(color);
        }

        marker_ids_scan0.pose.position.x = cluster_nn->centroid->x;
        marker_ids_scan0.pose.position.y = cluster_nn->centroid->y;
        marker_ids_scan0.pose.position.z = 5;

        //text
        boost::format fm("%d");
        fm % cluster_nn->id;

        marker_ids_scan0.text = fm.str();
        marker_ids_scan0.id = cluster_nn->id;

        marker_list.update(marker_ids_scan0);
        marker_list.update(marker_clusters_scan0);

    } //end for

    //Cluster from scan1
    vector<ClusterPtr> clusters_scan1 = sickLidarClusters_nn[1]->Clusters;

    for ( uint i = 0; i<  clusters_scan1.size() ; i++)
    {
        ClusterPtr cluster_nn = clusters_scan1[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_nn->support_points.size();h++)
        {
            geometry_msgs::Point pt;
            pt.x=cluster_nn->support_points[h]->x;
            pt.y=cluster_nn->support_points[h]->y;
            pt.z=cluster_nn->support_points[h]->z;

            marker_clusters_scan1.points.push_back(pt);
            marker_clusters_scan1.colors.push_back(color);
        }

        marker_ids_scan1.pose.position.x = cluster_nn->centroid->x;
        marker_ids_scan1.pose.position.y = cluster_nn->centroid->y;
        marker_ids_scan1.pose.position.z = 5;

        //text
        boost::format fm("%d");
        fm % cluster_nn->id;

        marker_ids_scan1.text = fm.str();
        marker_ids_scan1.id = cluster_nn->id;

        marker_list.update(marker_ids_scan1);
        marker_list.update(marker_clusters_scan1);

    } //end for

    //Cluster from scan2
    vector<ClusterPtr> clusters_scan2 = sickLidarClusters_nn[2]->Clusters;

    for ( uint i = 0; i<  clusters_scan2.size() ; i++)
    {
        ClusterPtr cluster_nn = clusters_scan2[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_nn->support_points.size();h++)
        {
            geometry_msgs::Point pt;
            pt.x=cluster_nn->support_points[h]->x;
            pt.y=cluster_nn->support_points[h]->y;
            pt.z=cluster_nn->support_points[h]->z;

            marker_clusters_scan2.points.push_back(pt);
            marker_clusters_scan2.colors.push_back(color);
        }

        marker_ids_scan2.pose.position.x = cluster_nn->centroid->x;
        marker_ids_scan2.pose.position.y = cluster_nn->centroid->y;
        marker_ids_scan2.pose.position.z = 3.3;

        //text
        boost::format fm("%d");
        fm % cluster_nn->id;

        marker_ids_scan2.text = fm.str();
        marker_ids_scan2.id = cluster_nn->id;

        marker_list.update(marker_ids_scan2);
        marker_list.update(marker_clusters_scan2);

    } //end for

    //Cluster from scan3
    vector<ClusterPtr> clusters_scan3 = sickLidarClusters_nn[3]->Clusters;

    for ( uint i = 0; i<  clusters_scan3.size() ; i++)
    {
        ClusterPtr cluster_nn = clusters_scan3[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_nn->support_points.size();h++)
        {
            geometry_msgs::Point pt;
            pt.x=cluster_nn->support_points[h]->x;
            pt.y=cluster_nn->support_points[h]->y;
            pt.z=cluster_nn->support_points[h]->z;

            marker_clusters_scan3.points.push_back(pt);
            marker_clusters_scan3.colors.push_back(color);
        }

        marker_ids_scan3.pose.position.x = cluster_nn->centroid->x;
        marker_ids_scan3.pose.position.y = cluster_nn->centroid->y;
        marker_ids_scan3.pose.position.z = 4.3;

        //text
        boost::format fm("%d");
        fm % cluster_nn->id;

        marker_ids_scan3.text = fm.str();
        marker_ids_scan3.id = cluster_nn->id;

        marker_list.update(marker_ids_scan3);
        marker_list.update(marker_clusters_scan3);

    } //end for

    //circle fit scan0

    vector<ClusterPtr> circles_scan0 = circlePoints[0]->Clusters;
    for ( uint i = 0; i< circles_scan0.size() ; i++)
    {
        ClusterPtr cluster_circle = circles_scan0[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_circle->support_points.size();h++)
        {
            geometry_msgs::Point pt;
            pt.x=cluster_circle->support_points[h]->x;
            pt.y=cluster_circle->support_points[h]->y;
            pt.z= -0.2;

            marker_clusters_circle0.points.push_back(pt);
            //marker_clusters_circle0.colors.push_back(color);
        }

        marker_ids_circle0.pose.position.x = cluster_circle->centroid->x;
        marker_ids_circle0.pose.position.y = cluster_circle->centroid->y;
        marker_ids_circle0.pose.position.z = 1.5;

        //text
        boost::format fm("%d");
        fm % 1.0;

        marker_ids_circle0.text = fm.str();
        marker_ids_circle0.id = 1.0;

        marker_list.update(marker_ids_circle0);
        marker_list.update(marker_clusters_circle0);

    } //end for

    //circle fit scan1

    vector<ClusterPtr> circles_scan1 = circlePoints[0]->Clusters;
    for ( uint i = 0; i< circles_scan1.size() ; i++)
    {
        ClusterPtr cluster_circle = circles_scan1[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_circle->support_points.size();h++)
        {
            geometry_msgs::Point pt;
            pt.x=cluster_circle->support_points[h]->x;
            pt.y=cluster_circle->support_points[h]->y;
            pt.z= -0.1;

            marker_clusters_circle1.points.push_back(pt);
            //marker_clusters_circle1.colors.push_back(color);
        }

        marker_ids_circle1.pose.position.x = cluster_circle->centroid->x;
        marker_ids_circle1.pose.position.y = cluster_circle->centroid->y;
        marker_ids_circle1.pose.position.z = 2.5;

        //text
        boost::format fm("%d");
        fm % 1.0;

        marker_ids_circle1.text = fm.str();
        marker_ids_circle1.id = 1.0;

        marker_list.update(marker_ids_circle1);
        marker_list.update(marker_clusters_circle1);

    } //end for

    //circle fit scan2

    vector<ClusterPtr> circles_scan2 = circlePoints[0]->Clusters;
    for ( uint i = 0; i< circles_scan0.size() ; i++)
    {
        ClusterPtr cluster_circle = circles_scan2[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_circle->support_points.size();h++)
        {
            geometry_msgs::Point pt;
            pt.x=cluster_circle->support_points[h]->x;
            pt.y=cluster_circle->support_points[h]->y;
            pt.z= 0.1;

            marker_clusters_circle2.points.push_back(pt);
            //marker_clusters_circle2.colors.push_back(color);
        }

        marker_ids_circle2.pose.position.x = cluster_circle->centroid->x;
        marker_ids_circle2.pose.position.y = cluster_circle->centroid->y;
        marker_ids_circle2.pose.position.z = 3.5;

        //text
        boost::format fm("%d");
        fm % 1.0;

        marker_ids_circle2.text = fm.str();
        marker_ids_circle2.id = 1.0;

        marker_list.update(marker_ids_circle2);
        marker_list.update(marker_clusters_circle2);

    } //end for

    //circle fit scan3

    vector<ClusterPtr> circles_scan3 = circlePoints[0]->Clusters;
    for ( uint i = 0; i< circles_scan3.size() ; i++)
    {
        ClusterPtr cluster_circle = circles_scan3[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_circle->support_points.size();h++)
        {
            geometry_msgs::Point pt;
            pt.x=cluster_circle->support_points[h]->x;
            pt.y=cluster_circle->support_points[h]->y;
            pt.z= 0.2;

            marker_clusters_circle3.points.push_back(pt);
            //marker_clusters_circle3.colors.push_back(color);
        }

        marker_ids_circle3.pose.position.x = cluster_circle->centroid->x;
        marker_ids_circle3.pose.position.y = cluster_circle->centroid->y;
        marker_ids_circle3.pose.position.z = 4.5;

        //text
        boost::format fm("%d");
        fm % 1.0;

        marker_ids_circle3.text = fm.str();
        marker_ids_circle3.id = 1.0;

        marker_list.update(marker_ids_circle3);
        marker_list.update(marker_clusters_circle3);

    } //end for

    //sphere

    if(sphere.x!=0)
    {
        std_msgs::ColorRGBA color = colormap.color(0);

        geometry_msgs::Point pt;
        pt.x=sphere.x;
        pt.y= sphere.y;
        pt.z= sphere.z;
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
