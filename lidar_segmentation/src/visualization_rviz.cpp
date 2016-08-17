/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
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
\file  visualization_rviz.cpp 
\brief Visualization on rviz related functions
\author Daniel Coimbra
*/

#include "lidar_segmentation/lidar_segmentation.h"
#include "lidar_segmentation/clustering.h"
#include "lidar_segmentation/visualization_rviz.h"

vector<visualization_msgs::Marker> createTargetMarkers( vector<ClusterPtr>& clusters , vector<ClusterPtr>& clusters_Premebida ,  vector<ClusterPtr>& clusters_Dietmayer, vector<ClusterPtr>& clusters_Santos, vector<ClusterPtr>& clusters_ABD ,vector<ClusterPtr>& clusters_nn , vector<ClusterPtr>&  clusters_GT  )
{		
	
	static Markers marker_list;
	
	//Reduce the elements status, ADD to REMOVE and REMOVE to delete
	marker_list.decrement();
	
	// Create a colormap
	class_colormap colormap("hsv",10, 1, false);
	
	visualization_msgs::Marker marker_ids;
	visualization_msgs::Marker marker_ids_prem;
	visualization_msgs::Marker marker_ids_diet;
	visualization_msgs::Marker marker_ids_santos;
	visualization_msgs::Marker marker_ids_abd;
	visualization_msgs::Marker marker_ids_nn;
	visualization_msgs::Marker marker_ids_gt;
	
	visualization_msgs::Marker marker_clusters;
	visualization_msgs::Marker marker_clusters_prem;
	visualization_msgs::Marker marker_clusters_diet;
	visualization_msgs::Marker marker_clusters_santos;
	visualization_msgs::Marker marker_clusters_abd;
	visualization_msgs::Marker marker_clusters_nn;
	visualization_msgs::Marker marker_clusters_gt;
	
	marker_ids.header.frame_id = "/my_frame";
	marker_ids.header.stamp = ros::Time::now();
	marker_ids_prem.header.frame_id = "/my_frame";
	marker_ids_prem.header.stamp = ros::Time::now();
	marker_ids_diet.header.frame_id = "/my_frame";
	marker_ids_diet.header.stamp = ros::Time::now();
	marker_ids_santos.header.frame_id = "/my_frame";
	marker_ids_santos.header.stamp = ros::Time::now();
	marker_ids_abd.header.frame_id = "/my_frame";
	marker_ids_abd.header.stamp = ros::Time::now();
	marker_ids_nn.header.frame_id = "/my_frame";
	marker_ids_nn.header.stamp = ros::Time::now();
	marker_ids_gt.header.frame_id = "/my_frame";
	marker_ids_gt.header.stamp = ros::Time::now();
	
	marker_clusters.header.frame_id = "/my_frame";
    marker_clusters.header.stamp = marker_ids.header.stamp;
	marker_clusters_prem.header.frame_id = "/my_frame";
	marker_clusters_prem.header.stamp = marker_ids_prem.header.stamp;
	marker_clusters_diet.header.frame_id = "/my_frame";
	marker_clusters_diet.header.stamp = marker_ids_diet.header.stamp;
	marker_clusters_santos.header.frame_id = "/my_frame";
	marker_clusters_santos.header.stamp = marker_ids_santos.header.stamp;
	marker_clusters_abd.header.frame_id = "/my_frame";
	marker_clusters_abd.header.stamp = marker_ids_abd.header.stamp;
	marker_clusters_nn.header.frame_id = "/my_frame";
	marker_clusters_nn.header.stamp = marker_ids_nn.header.stamp;
	marker_clusters_gt.header.frame_id = "/my_frame";
	marker_clusters_gt.header.stamp = marker_ids_gt.header.stamp;
	
	marker_ids.ns = "ids";
	marker_ids.action = visualization_msgs::Marker::ADD;
	marker_ids_prem.ns = "ids_prem";
	marker_ids_prem.action = visualization_msgs::Marker::ADD;
	marker_ids_diet.ns = "ids_diet";
	marker_ids_diet.action = visualization_msgs::Marker::ADD;
	marker_ids_santos.ns = "ids_santos";
	marker_ids_santos.action = visualization_msgs::Marker::ADD;
	marker_ids_abd.ns = "ids_abd";
	marker_ids_abd.action = visualization_msgs::Marker::ADD;
	marker_ids_nn.ns = "ids_nn";
	marker_ids_nn.action = visualization_msgs::Marker::ADD;
	marker_ids_gt.ns = "ids_gt";
	marker_ids_gt.action = visualization_msgs::Marker::ADD;	
	
	marker_clusters.ns = "clusters";
	marker_clusters.action = visualization_msgs::Marker::ADD;
	marker_clusters_prem.ns = "clusters_prem";
	marker_clusters_prem.action = visualization_msgs::Marker::ADD;
	marker_clusters_diet.ns = "clusters_diet";
	marker_clusters_diet.action = visualization_msgs::Marker::ADD;
	marker_clusters_santos.ns = "clusters_santos";
	marker_clusters_santos.action = visualization_msgs::Marker::ADD;
	marker_clusters_abd.ns = "clusters_abd";
	marker_clusters_abd.action = visualization_msgs::Marker::ADD;
	marker_clusters_nn.ns = "clusters_nn";
	marker_clusters_nn.action = visualization_msgs::Marker::ADD;
	marker_clusters_gt.ns = "clusters_gt";
	marker_clusters_gt.action = visualization_msgs::Marker::ADD;
	
	marker_ids.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker_ids_prem.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker_ids_diet.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker_ids_santos.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker_ids_abd.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker_ids_nn.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker_ids_gt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	
	marker_clusters.type = visualization_msgs::Marker::SPHERE_LIST;	
	marker_clusters_prem.type = visualization_msgs::Marker::SPHERE_LIST;
	marker_clusters_diet.type = visualization_msgs::Marker::SPHERE_LIST;
	marker_clusters_santos.type = visualization_msgs::Marker::SPHERE_LIST;
	marker_clusters_abd.type = visualization_msgs::Marker::SPHERE_LIST;
	marker_clusters_nn.type = visualization_msgs::Marker::SPHERE_LIST;
	marker_clusters_gt.type = visualization_msgs::Marker::SPHERE_LIST;
	
	marker_ids.scale.x = 0.5;
	marker_ids.scale.y = 0.5;
	marker_ids.scale.z = 0.5;
	marker_ids_prem.scale.x = 0.5;
	marker_ids_prem.scale.y = 0.5;
	marker_ids_prem.scale.z = 0.5;
	marker_ids_diet.scale.x = 0.5;
	marker_ids_diet.scale.y = 0.5;
	marker_ids_diet.scale.z = 0.5;
	marker_ids_santos.scale.x = 0.5;
	marker_ids_santos.scale.y = 0.5;
	marker_ids_santos.scale.z = 0.5;
	marker_ids_abd.scale.x = 0.5;
	marker_ids_abd.scale.y = 0.5;
	marker_ids_abd.scale.z = 0.5;
	marker_ids_nn.scale.x = 0.5;
	marker_ids_nn.scale.y = 0.5;
	marker_ids_nn.scale.z = 0.5;
	marker_ids_gt.scale.x = 0.5;
	marker_ids_gt.scale.y = 0.5;
	marker_ids_gt.scale.z = 0.5;
	
	marker_clusters.scale.x = 0.2;
	marker_clusters.scale.y = 0.2;
	marker_clusters.scale.z = 0.2;	
	marker_clusters_prem.scale.x = 0.2;
	marker_clusters_prem.scale.y = 0.2;
	marker_clusters_prem.scale.z = 0.2;
	marker_clusters_diet.scale.x = 0.2;
	marker_clusters_diet.scale.y = 0.2;
	marker_clusters_diet.scale.z = 0.2;
	marker_clusters_santos.scale.x = 0.2;
	marker_clusters_santos.scale.y = 0.2;
	marker_clusters_santos.scale.z = 0.2;
	marker_clusters_abd.scale.x = 0.2;
	marker_clusters_abd.scale.y = 0.2;
	marker_clusters_abd.scale.z = 0.2;
	marker_clusters_nn.scale.x = 0.2;
	marker_clusters_nn.scale.y = 0.2;
	marker_clusters_nn.scale.z = 0.2;
	marker_clusters_gt.scale.x = 0.2;
	marker_clusters_gt.scale.y = 0.2;
	marker_clusters_gt.scale.z = 0.2;	
	
	marker_ids.color.a = 1.0;    
    marker_ids.color.r = 0.0;   
    marker_ids.color.g = 0.0;
    marker_ids.color.b = 0.0;
	marker_ids_prem.color.a = 1.0;    
    marker_ids_prem.color.r = 0.0;   
    marker_ids_prem.color.g = 0.0;
    marker_ids_prem.color.b = 0.0;
	marker_ids_diet.color.a = 1.0;    
    marker_ids_diet.color.r = 0.0;   
    marker_ids_diet.color.g = 0.0;
    marker_ids_diet.color.b = 0.0;
	marker_ids_santos.color.a = 1.0;    
    marker_ids_santos.color.r = 0.0;   
    marker_ids_santos.color.g = 0.0;
    marker_ids_santos.color.b = 0.0;
	marker_ids_abd.color.a = 1.0;    
    marker_ids_abd.color.r = 0.0;   
    marker_ids_abd.color.g = 0.0;
    marker_ids_abd.color.b = 0.0;
	marker_ids_nn.color.a = 1.0;    
    marker_ids_nn.color.r = 0.0;   
    marker_ids_nn.color.g = 0.0;
    marker_ids_nn.color.b = 0.0;
	marker_ids_gt.color.a = 1.0;    
    marker_ids_gt.color.r = 0.0;   
    marker_ids_gt.color.g = 0.0;
    marker_ids_gt.color.b = 0.0;

	
	for ( uint i = 0 ; i< clusters.size() ; i++)  //search all clusters
	{	
		ClusterPtr cluster = clusters[i];
		
		std_msgs::ColorRGBA color = colormap.color(i);
		
		//Place in the marker every point belonging to the cluster "i"		
		for(uint h=0;h<cluster->support_points.size();h++)
		{
			geometry_msgs::Point pt;
			pt.x=cluster->support_points[h]->x;
			pt.y=cluster->support_points[h]->y;
			pt.z=0;
			
			marker_clusters.points.push_back(pt);
			marker_clusters.colors.push_back(color);
		}
					
		marker_ids.pose.position.x = cluster->centroid->x;
		marker_ids.pose.position.y = cluster->centroid->y;
		marker_ids.pose.position.z = 0.3;
				
		//texto
		boost::format fm("%d");
		fm	% cluster->id;
		
		marker_ids.text = fm.str();
		marker_ids.id = cluster->id;
		marker_list.update(marker_ids);	
		
		marker_list.update(marker_clusters);
		
	} //end for
	
	
	for ( uint i = 0; i< clusters_Premebida.size() ; i++)
	{
		
		ClusterPtr cluster_prem = clusters_Premebida[i];

		std_msgs::ColorRGBA color = colormap.color(i);
		
		//Place in the marker every point belonging to the cluster "i"		
		for(uint h=0;h<cluster_prem->support_points.size();h++)
		{
			geometry_msgs::Point pt;
			pt.x=cluster_prem->support_points[h]->x;
			pt.y=cluster_prem->support_points[h]->y;
			pt.z= 1.0;
			
			marker_clusters_prem.points.push_back(pt);
			marker_clusters_prem.colors.push_back(color);
		}
		
		marker_ids_prem.pose.position.x = cluster_prem->centroid->x;
		marker_ids_prem.pose.position.y = cluster_prem->centroid->y;
		marker_ids_prem.pose.position.z = 1.3; 
		
// 		text
		boost::format fm("%d");
		fm % cluster_prem->id;
		
		marker_ids_prem.text = fm.str();
		marker_ids_prem.id = cluster_prem->id;
		
		marker_list.update(marker_ids_prem);
		marker_list.update(marker_clusters_prem);
		
	} //end for
	
	for ( uint i = 0; i< clusters_Dietmayer.size() ; i++)
	{
 		ClusterPtr cluster_diet = clusters_Dietmayer[i];
		
		std_msgs::ColorRGBA color = colormap.color(i);
		
		//Place in the marker every point belonging to the cluster "i"		
		for(uint h=0;h<cluster_diet->support_points.size();h++)
		{
			geometry_msgs::Point pt;
			pt.x=cluster_diet->support_points[h]->x;
			pt.y=cluster_diet->support_points[h]->y;
			pt.z= 2.0;
			
			marker_clusters_diet.points.push_back(pt);
			marker_clusters_diet.colors.push_back(color);
		}
		
		marker_ids_diet.pose.position.x = cluster_diet->centroid->x;
		marker_ids_diet.pose.position.y = cluster_diet->centroid->y;
		marker_ids_diet.pose.position.z = 2.3; 
		
		//text
		boost::format fm("%d");
		fm % cluster_diet->id;
		
		marker_ids_diet.text = fm.str();
		marker_ids_diet.id = cluster_diet->id;
		
		marker_list.update(marker_ids_diet);
		marker_list.update(marker_clusters_diet);
		
	} //end for
	
	for ( uint i = 0; i< clusters_Santos.size() ; i++)
	{
 		ClusterPtr cluster_santos = clusters_Santos[i];
		
		std_msgs::ColorRGBA color = colormap.color(i);
		
		//Place in the marker every point belonging to the cluster "i"		
		for(uint h=0;h<cluster_santos->support_points.size();h++)
		{
			geometry_msgs::Point pt;
			pt.x=cluster_santos->support_points[h]->x;
			pt.y=cluster_santos->support_points[h]->y;
			pt.z= 3.0;
			
			marker_clusters_santos.points.push_back(pt);
			marker_clusters_santos.colors.push_back(color);
		}
		
		marker_ids_santos.pose.position.x = cluster_santos->centroid->x;
		marker_ids_santos.pose.position.y = cluster_santos->centroid->y;
		marker_ids_santos.pose.position.z = 3.3; 
		
		//text
		boost::format fm("%d");
		fm % cluster_santos->id;
		
		marker_ids_santos.text = fm.str();
		marker_ids_santos.id = cluster_santos->id;
		
		marker_list.update(marker_ids_santos);
		marker_list.update(marker_clusters_santos);
		
	} //end for
	
	for ( uint i = 0; i< clusters_ABD.size() ; i++)
	{
		ClusterPtr cluster_abd = clusters_ABD[i];
		
		std_msgs::ColorRGBA color = colormap.color(i);
		
		//Place in the marker every point belonging to the cluster "i"		
		for(uint h=0;h<cluster_abd->support_points.size();h++)
		{
			geometry_msgs::Point pt;
			pt.x=cluster_abd->support_points[h]->x;
			pt.y=cluster_abd->support_points[h]->y;
			pt.z= 4.0;
			
			marker_clusters_abd.points.push_back(pt);
			marker_clusters_abd.colors.push_back(color);
		}
		
		marker_ids_abd.pose.position.x = cluster_abd->centroid->x;
		marker_ids_abd.pose.position.y = cluster_abd->centroid->y;
		marker_ids_abd.pose.position.z = 4.3; 
		
		//text
		boost::format fm("%d");
		fm % cluster_abd->id;
		
		marker_ids_abd.text = fm.str();
		marker_ids_abd.id = cluster_abd->id;
		
		marker_list.update(marker_ids_abd);
		marker_list.update(marker_clusters_abd);
		
	} //end for
	
	for ( uint i = 0; i< clusters_nn.size() ; i++)
	{
 		ClusterPtr cluster_nn = clusters_nn[i];

		std_msgs::ColorRGBA color = colormap.color(i);
		
		//Place in the marker every point belonging to the cluster "i"		
		for(uint h=0;h<cluster_nn->support_points.size();h++)
		{
			geometry_msgs::Point pt;
			pt.x=cluster_nn->support_points[h]->x;
			pt.y=cluster_nn->support_points[h]->y;
			pt.z= 5.0;
			
			marker_clusters_nn.points.push_back(pt);
			marker_clusters_nn.colors.push_back(color);
		}
		
		marker_ids_nn.pose.position.x = cluster_nn->centroid->x;
		marker_ids_nn.pose.position.y = cluster_nn->centroid->y;
		marker_ids_nn.pose.position.z = 5.3; 
		
		//text
		boost::format fm("%d");
		fm % cluster_nn->id;
		
		marker_ids_nn.text = fm.str();
		marker_ids_nn.id = cluster_nn->id;
		
		marker_list.update(marker_ids_nn);
		marker_list.update(marker_clusters_nn);
		
	} //end for
	
	for ( uint i = 0; i< clusters_GT.size() ; i++)
	{
 		ClusterPtr cluster_gt = clusters_GT[i];

		std_msgs::ColorRGBA color = colormap.color(i);
		
		//Place in the marker every point belonging to the cluster "i"		
		for(uint h=0;h<cluster_gt->support_points.size();h++)
		{
			geometry_msgs::Point pt;
			pt.x=cluster_gt->support_points[h]->x;
			pt.y=cluster_gt->support_points[h]->y;
			pt.z= 6.0;
			
			marker_clusters_gt.points.push_back(pt);
			marker_clusters_gt.colors.push_back(color);
		}
		
		marker_ids_gt.pose.position.x = cluster_gt->centroid->x;
		marker_ids_gt.pose.position.y = cluster_gt->centroid->y;
		marker_ids_gt.pose.position.z = 6.3; 
			
		//text
		boost::format fm("%d");
		fm % cluster_gt->id;
			
		marker_ids_gt.text = fm.str();
		marker_ids_gt.id = cluster_gt->id;
			
		marker_list.update(marker_ids_gt);
		marker_list.update(marker_clusters_gt);
		
	} //end for
	
	
	//Remove markers that should not be transmitted
	marker_list.clean();
	
	//Clean the marker_vector and put new markers in it;
	return marker_list.getOutgoingMarkers();
	
} //end function
