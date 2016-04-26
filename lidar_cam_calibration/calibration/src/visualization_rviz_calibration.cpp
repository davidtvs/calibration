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
 \file  visualization_rviz_calibration.cpp
 \brief Representation of the calibration result
 \details The resulting calibration of the sensors is represented by arrows, as well as the point clouds acquired during the calibration process
 \author Marcelo Pereira
 \date   December, 2015
*/

#include <lidar_segmentation/lidar_segmentation.h>
#include <lidar_segmentation/clustering.h>
#include "calibration/visualization_rviz_calibration.h"
#include <visualization_msgs/Marker.h>

/**
@brief Markers publication for the visualization of the calibration ressult on rviz
@param[in] clouds ball center acquisitions in all sensors
@param[in] lasers position of the sensors
@return vector<visualization_msgs::Marker>
*/
vector<visualization_msgs::Marker> createTargetMarkers(vector<pcl::PointCloud<pcl::PointXYZ> > clouds, vector<geometry_msgs::Pose> lasers,
                                                       const vector<double>& RPY, const vector<double>& translation )
{
  tf::Transform t_rpy;
  tf::Quaternion q;

  if (translation.empty() && RPY.empty()) //user does not want to translate/rotate clouds and sensors.
  {
    t_rpy.setOrigin( tf::Vector3 (tfScalar(0), tfScalar(0), tfScalar(0)) ); // no translation is done
    q = tf::createQuaternionFromRPY(0.0, 0.0, 0.0 ); // no rotation
  	t_rpy.setRotation( q );
  }
  else if (translation.empty()) // only rotation given by the user, no translation
  {
    t_rpy.setOrigin( tf::Vector3 (tfScalar(0), tfScalar(0), tfScalar(0)) ); // no translation
    q = tf::createQuaternionFromRPY( RPY[0], RPY[1], RPY[2] ); // quaternion computation from given angles
  	t_rpy.setRotation( q );
  }
  else // rotation and translation given by the user
  {
  	t_rpy.setOrigin( tf::Vector3 (tfScalar(translation[0]), tfScalar(translation[1]), tfScalar(translation[2])) ); // translation given by the user
  	q = tf::createQuaternionFromRPY( RPY[0], RPY[1], RPY[2] ); // quaternion computation from given angles
  	t_rpy.setRotation( q );
  }

	static Markers marker_list;
	int nLasers=lasers.size();

	//Reduce the elements status, ADD to REMOVE and REMOVE to delete
	marker_list.decrement();

	// Create a colormap
	class_colormap colormap("hsv",10, 1, false);

	visualization_msgs::Marker marker_centers;
	visualization_msgs::Marker marker_lasers[nLasers];

	marker_centers.header.frame_id = "/my_frame3";
	marker_centers.header.stamp = ros::Time::now();

	marker_centers.ns = "centers";
	marker_centers.action = visualization_msgs::Marker::ADD;

	marker_centers.type = visualization_msgs::Marker::SPHERE_LIST;

	marker_centers.scale.x = 0.08;
	marker_centers.scale.y = 0.08;
	marker_centers.scale.z = 0.08;

	marker_centers.pose.orientation.x=q[0];
	marker_centers.pose.orientation.y=q[1];
	marker_centers.pose.orientation.z=q[2];
	marker_centers.pose.orientation.w=q[3];

	/*marker_centers.color.b=1;
	   marker_centers.color.a=1;*/
	marker_lasers[0].color.a=1;
	marker_lasers[0].color.g=1;
	marker_lasers[1].color.b=1;
	marker_lasers[1].color.a=1;
	marker_lasers[2].color.r=1;
	marker_lasers[2].color.a=1;

	for(int n=0; n<clouds.size(); n++)
	{
		{
			pcl::PointCloud<pcl::PointXYZ> cloud = clouds[n];
			std_msgs::ColorRGBA color;
			color = colormap.color(n);

			for ( int i = 0; i< cloud.points.size(); i++)
			{
				geometry_msgs::Point pt;
				pt.x= cloud.points[i].x;
				pt.y= cloud.points[i].y;
				pt.z= cloud.points[i].z;

				marker_centers.points.push_back(pt);
				marker_centers.colors.push_back(color);


			} //end for
			marker_list.update(marker_centers);
		}
	}

	//position and orientation of laser
	for(int n=0; n<lasers.size(); n++)
	{
		{
			marker_lasers[n].header.frame_id = "/my_frame3";
			marker_lasers[n].header.stamp = ros::Time::now();

			std::stringstream ss;
			ss << "Laser " << n;

			marker_lasers[n].ns = ss.str();
			marker_lasers[n].action = visualization_msgs::Marker::ADD;

			marker_lasers[n].type = visualization_msgs::Marker::ARROW;

			marker_lasers[n].scale.x = 0.2;
			marker_lasers[n].scale.y = 0.1;
			marker_lasers[n].scale.z = 0.1;

			geometry_msgs::Pose laser = lasers[n];
			std_msgs::ColorRGBA color;

			color = colormap.color(n);

			tf::Transform t_laser;
			t_laser.setOrigin( tf::Vector3(laser.position.x, laser.position.y, laser.position.z) );
			tf::Quaternion q_laser(laser.orientation.x, laser.orientation.y, laser.orientation.z, laser.orientation.w);
			t_laser.setRotation(q_laser);

			t_laser= t_rpy * t_laser;
			q_laser = t_laser.getRotation();
			tf::Vector3 trans_laser = t_laser.getOrigin();

			marker_lasers[n].pose.position.x=trans_laser[0];
			marker_lasers[n].pose.position.y=trans_laser[1];
			marker_lasers[n].pose.position.z=trans_laser[2];

			marker_lasers[n].pose.orientation.x=q_laser[0];
			marker_lasers[n].pose.orientation.y=q_laser[1];
			marker_lasers[n].pose.orientation.z=q_laser[2];
			marker_lasers[n].pose.orientation.w=q_laser[3];

			marker_lasers[n].color=color;

			marker_list.update(marker_lasers[n]);
		}
	}

	//Remove markers that should not be transmitted
	marker_list.clean();

	//Clean the marker_vector and put new markers in it;
	return marker_list.getOutgoingMarkers();

} //end function
