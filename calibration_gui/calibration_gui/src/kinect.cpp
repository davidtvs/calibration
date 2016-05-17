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
   \file  kinect.cpp
   \brief Algorithm for the ball center detection with Kinect based on swissranger
   \author David Silva
   \date   May, 2016
 */

#include "calibration_gui/kinect.h"
#include "calibration_gui/visualization_rviz_kinect.h"
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/normal_3d.h>


ros::Publisher markers_pub;
ros::Publisher sphereCenter_pub;
ros::Publisher pointCloud_pub;

pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");


/**
   @brief write in a file the center of the sphere
   @param[in] sphereCoeffsRefined center sphere coordinates
   @return void
 */
void writeFile(Eigen::VectorXf sphereCoeffsRefined)
{
	FILE* pFile = fopen("sr_3m.txt", "a");
	fprintf(pFile, "%F %F %F\n ", sphereCoeffsRefined(0),sphereCoeffsRefined(1),sphereCoeffsRefined(2));
	fclose(pFile);
}

/**
   @brief Detection of the ball on the sensor data
   @param[in] SwissRanger_cloud point cloud from the swissranger
   @return void
 */
void sphereDetection(pcl::PointCloud<pcl::PointXYZ> Kinect_cloud)
{

	ros::Time start = ros::Time::now();

	geometry_msgs::PointStamped sphereCenter;
	sphereCenter.point.x = -999;
	sphereCenter.point.y = -999;
	sphereCenter.point.z = -999;


	pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(Kinect_cloud));

	pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "PointCloud before filtering: " << Kinect_cloudPtr->width * Kinect_cloudPtr->height << " data points." << std::endl;

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (Kinect_cloudPtr);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*Kinect_cloud_filtered);

	viewer.showCloud(Kinect_cloud_filtered);

  std::cerr << "PointCloud after filtering: " << Kinect_cloud_filtered->width * Kinect_cloud_filtered->height << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_SPHERE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.02);
  seg.setRadiusLimits (BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2, BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2);

	seg.setInputCloud (Kinect_cloud_filtered);
	seg.segment (*inliers, *coefficients);

	ros::Time end = ros::Time::now();

	cout << "Ball detection time filtered: " <<  (end - start).toNSec() * 1e-6 << " msec"  << endl;

	if(inliers->indices.size ()>0)
	{

		if (coefficients->values[3]<BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2 && coefficients->values[3]>BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2)
		{
			sphereCenter.point.x = coefficients->values[0];
			sphereCenter.point.y = coefficients->values[1];
			sphereCenter.point.z = coefficients->values[2];
		}
		cout << coefficients->values[0] << ", "
		     << coefficients->values[1] << ", "
		     << coefficients->values[2] << ", "
		     << coefficients->values[3] << endl;
	}

	start = ros::Time::now();

	/*
	 * Source: http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_3:_Cloud_processing_%28advanced%29
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(Kinect_cloud));
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ModelCoefficients::Ptr coefficients_(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloudPtr);
	segmentation.setModelType(pcl::SACMODEL_SPHERE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.02);
	segmentation.setRadiusLimits(BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2, BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2);
	segmentation.setOptimizeCoefficients(true);

	pcl::PointIndices inlierIndices;
	segmentation.segment(inlierIndices, *coefficients_);

	end = ros::Time::now();

	if(inliers->indices.size ()>0)
	{

		if (coefficients_->values[3]<BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2 && coefficients_->values[3]>BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2)
		{
			sphereCenter.point.x = coefficients_->values[0];
			sphereCenter.point.y = coefficients_->values[1];
			sphereCenter.point.z = coefficients_->values[2];
		}
		cout << coefficients_->values[0] << ", "
		     << coefficients_->values[1] << ", "
		     << coefficients_->values[2] << ", "
		     << coefficients_->values[3] << endl;
	}

	cout << "Ball detection time full: " <<  (end - start).toNSec() * 1e-6 << " msec"  << endl;



	// sphereCenter.header.stamp = ros::Time::now();
	// sphereCenter_pub.publish(sphereCenter);
	//
	// pcl::PointXYZ center;
	// center.x = sphereCenter.point.x;
	// center.y = sphereCenter.point.y;
	// center.z = sphereCenter.point.z;
	//
	// visualization_msgs::MarkerArray targets_markers;
	// targets_markers.markers = createTargetMarkers(center);
	// markers_pub.publish(targets_markers);

}

/**
   @brief Main function of the swissranger node
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinect");

	ros::NodeHandle n("~");
	string sub_node_name;
	n.getParam("sub_node_name", sub_node_name);
	n.getParam("ballDiameter", BALL_DIAMETER);
	cout << "Subscribe node:" << sub_node_name << endl;
	cout << "Ball diameter:" << BALL_DIAMETER << endl;

	std::cout << sub_node_name << std::endl;

	markers_pub = n.advertise<visualization_msgs::MarkerArray>( "BallDetection", 10000);
	sphereCenter_pub = n.advertise<geometry_msgs::PointStamped>("SphereCentroid",1000);
	pointCloud_pub = n.advertise<sensor_msgs::PointCloud2>("Pointcloud",10000);
	sensor_msgs::PointCloud2 rosPointCloud2;

	kinect cloud(sub_node_name);

	ros::Rate loop_rate(30);

	while(ros::ok())
	{
		//if(cloud.cloud->points.size()>0)
		//{

			sphereDetection(cloud.cloud);


			pcl::toROSMsg(cloud.cloud, rosPointCloud2);
			rosPointCloud2.header.frame_id = "/my_frame";
			rosPointCloud2.header.stamp = ros::Time::now();
			pointCloud_pub.publish(rosPointCloud2);
		//}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
