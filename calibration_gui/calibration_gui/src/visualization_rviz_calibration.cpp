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
   \file  visualization_rviz_calibration.cpp
   \brief Representation of the calibration result
   \details The resulting calibration of the sensors is represented by arrows, as well as the point clouds acquired during the calibration process
   \author Marcelo Pereira, David Silva
   \date   July, 2016
 */

#include <lidar_segmentation/lidar_segmentation.h>
#include <lidar_segmentation/clustering.h>
#include "calibration_gui/visualization_rviz_calibration.h"
#include <visualization_msgs/Marker.h>

/**
   @brief Markers publication for the visualization of the calibration ressult on rviz
   @param[in] clouds ball center acquisitions in all sensors
   @param[in] lasers position of the sensors
   @return vector<visualization_msgs::Marker>
 */
vector<visualization_msgs::Marker> createTargetMarkers(vector<pcl::PointCloud<pcl::PointXYZ> > clouds, vector<geometry_msgs::Pose> lasers, vector<string> deviceNames)
{

	static Markers marker_list;
	int nLasers=lasers.size();
	std::cout << "number of lasers: " << nLasers << std::endl;

	//Reduce the elements status, ADD to REMOVE and REMOVE to delete
	marker_list.decrement();

	// Create a colormap
	class_colormap colormap("hsv", nLasers, 1, false);

	visualization_msgs::Marker marker_centers;
	visualization_msgs::Marker marker_lasers[nLasers*5]; // *4 to make space for Y, Z axis and square that represents the laser

	marker_centers.header.frame_id = "/my_frame3";
	marker_centers.header.stamp = ros::Time::now();

	marker_centers.ns = "centers";
	marker_centers.id = 0;
	marker_centers.action = visualization_msgs::Marker::ADD;

	marker_centers.type = visualization_msgs::Marker::SPHERE_LIST;

	marker_centers.scale.x = 0.08;
	marker_centers.scale.y = 0.08;
	marker_centers.scale.z = 0.08;


	marker_lasers[0].color.a=1;
	marker_lasers[0].color.g=1;
	marker_lasers[1].color.b=1;
	marker_lasers[1].color.a=1;
	marker_lasers[2].color.r=1;
	marker_lasers[2].color.a=1;

	for(int n=0; n<clouds.size(); n++)
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


		}         //end for
		marker_list.update(marker_centers);
	}

	//position and orientation of laser - David Silva
	int counter = 0;
	for(int n=0; n<lasers.size(); n++)
	{

		geometry_msgs::Pose laser = lasers[n];
		tf::Transform t_laser;
		t_laser.setOrigin( tf::Vector3(laser.position.x, laser.position.y, laser.position.z) );
		tf::Quaternion q_laser(laser.orientation.x, laser.orientation.y, laser.orientation.z, laser.orientation.w);
		t_laser.setRotation(q_laser);

		for (int k=0; k<5; k++)
		{
			marker_lasers[counter].header.frame_id = "/my_frame3";
			marker_lasers[counter].header.stamp = ros::Time::now();

			std::stringstream ss;
			ss << "Laser " << n << "Marker: " << k;

			marker_lasers[counter].ns = ss.str();
			marker_lasers[counter].action = visualization_msgs::Marker::ADD;

			std_msgs::ColorRGBA color;

			if (k<3)
			{
				marker_lasers[counter].type = visualization_msgs::Marker::ARROW;

				marker_lasers[counter].scale.x = 0.5;
				marker_lasers[counter].scale.y = 0.05;
				marker_lasers[counter].scale.z = 0.05;

				color.r = 0;
				color.g = 0;
				color.b = 0;
				color.a = 1;
			}
			else if (k == 3)
			{
				marker_lasers[counter].type = visualization_msgs::Marker::CUBE;
				marker_lasers[counter].scale.x = 0.15;
				marker_lasers[counter].scale.y = 0.15;
				marker_lasers[counter].scale.z = 0.15;

			}
			else
			{
				marker_lasers[counter].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
				marker_lasers[counter].text = deviceNames[n];
				marker_lasers[counter].scale.x = 0.15;
				marker_lasers[counter].scale.y = 0.15;
				marker_lasers[counter].scale.z = 0.15;
				std::cout << deviceNames[n] << std::endl;
			}

			tf::Transform t_rpy;
			tf::Quaternion q;
			t_rpy.setOrigin( tf::Vector3 (tfScalar(0), tfScalar(0), tfScalar(0)) ); // no translation is done

			switch (k) {
			case 0:
			{
				q = tf::createQuaternionFromRPY(0.0, 0.0, 0.0 ); // no rotation->this is the X axis
				color.r = 1.0;
				//std::cout << "2 " << k << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << color << std::endl;
				break;
			}
			case 1:
			{
				q = tf::createQuaternionFromRPY( 0.0, 0.0, M_PI/2 ); // rotation to get Y axis from X
				color.g = 1.0;
				//std::cout << "2 " << k << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << color << std::endl;
				break;
			}
			case 2:
			{
				q = tf::createQuaternionFromRPY( 0.0, -M_PI/2, 0.0); // rotation to get Z axis from X
				color.b = 1.0;
				//std::cout << "2 " << k << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << color << std::endl;
				break;
			}
			case 3:
			{
				q = tf::createQuaternionFromRPY( 0.0, 0.0, 0.0); // no rotation->this is the square at the axes origin that represents the sensor
				color = colormap.color(n);
				//std::cout << "2 " << k << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << color << std::endl;
				break;
			}
			default:
			{
				std::cout << "text" << std::endl;
				q = tf::createQuaternionFromRPY( 0.0, 0.0, 0.0);  // no rotation->this is the square at the axes origin that represents the sensor
				color.r = 1.0;
				color.g = 1.0;
				color.b = 1.0;
				color.a = 1.0;
			}
			}

			t_rpy.setRotation( q );

			tf::Transform transform_laser = t_laser * t_rpy;
			tf::Quaternion rotation_laser = transform_laser.getRotation();
			tf::Vector3 translation_laser = transform_laser.getOrigin();

			//std::cout << "translation " << translation_laser[0] << " " << translation_laser[1] << " " << translation_laser[2] << std::endl;
			//std::cout << "rotation " << rotation_laser[0] << " " << rotation_laser[1] << " " << rotation_laser[2] << " " << rotation_laser[3] << std::endl;

			marker_lasers[counter].pose.position.y=translation_laser[1];
			if (k!=4)
			{
				marker_lasers[counter].pose.position.x=translation_laser[0];
				marker_lasers[counter].pose.position.z=translation_laser[2];
			}
			else
			{
				marker_lasers[counter].pose.position.x=translation_laser[0]+0.15;
				marker_lasers[counter].pose.position.z=translation_laser[2]+0.25;
			}

			marker_lasers[counter].pose.orientation.x=rotation_laser[0];
			marker_lasers[counter].pose.orientation.y=rotation_laser[1];
			marker_lasers[counter].pose.orientation.z=rotation_laser[2];
			marker_lasers[counter].pose.orientation.w=rotation_laser[3];

			marker_lasers[counter].color=color;

			marker_list.update(marker_lasers[counter]);
			counter++;
		}
	}

	//Remove markers that should not be transmitted
	marker_list.clean();

	//Clean the marker_vector and put new markers in it;
	return marker_list.getOutgoingMarkers();

}   //end function
