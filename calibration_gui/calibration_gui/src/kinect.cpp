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

typedef unsigned long long timestamp_t;

static timestamp_t get_timestamp ()
{
	struct timeval now;
	gettimeofday (&now, NULL);
	return now.tv_usec + (timestamp_t)now.tv_sec * 1000000;
}


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

	timestamp_t t0, t1;
	double secs;
	t0 = get_timestamp();

	geometry_msgs::PointStamped sphereCenter;
	sphereCenter.point.x = -999;
	sphereCenter.point.y = -999;
	sphereCenter.point.z = -999;


/*

	pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(Kinect_cloud));



	pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentation_from_normals;

pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


pcl::PointCloud<pcl::PointXYZ> Kinect_cloud_filtered;
pcl::VoxelGrid<pcl::PointXYZ> sor;
sor.setInputCloud (Kinect_cloudPtr);
sor.setLeafSize (0.1, 0.1, 0.1);
sor.filter (Kinect_cloud_filtered);

// Estimate point normals
  normal_estimation.setSearchMethod (tree);
  normal_estimation.setInputCloud (Kinect_cloudPtr);
  normal_estimation.setKSearch (25);
  normal_estimation.compute (*cloud_normals);

	// Create the segmentation object for sphere segmentation and set all the paopennirameters
segmentation_from_normals.setOptimizeCoefficients (true);
//segmentation_from_normals.setModelType (pcl::SACMODEL_SPHERE);
segmentation_from_normals.setModelType (pcl::SACMODEL_SPHERE);
segmentation_from_normals.setMethodType (pcl::SAC_RANSAC);
segmentation_from_normals.setNormalDistanceWeight (0.1);
segmentation_from_normals.setMaxIterations (1000);
segmentation_from_normals.setDistanceThreshold (0.05);
segmentation_from_normals.setRadiusLimits (0.45, 0.5);
segmentation_from_normals.setInputCloud (Kinect_cloudPtr);
segmentation_from_normals.setInputNormals (cloud_normals);


  // Obtain the sphere inliers and coefficients
  segmentation_from_normals.segment (*inliers, *coefficients);*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(Kinect_cloud));

	pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "PointCloud before filtering: " << Kinect_cloudPtr->width * Kinect_cloudPtr->height << " data points." << std::endl;

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (Kinect_cloudPtr);
	sor.setLeafSize (0.01, 0.01, 0.01);
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
  seg.setMaxIterations (500);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0.4, 0.6);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = Kinect_cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (Kinect_cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (Kinect_cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (Kinect_cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    if (cloud_p->points.size () < 1000)
      continue;
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		cout << coefficients->values[0] << ", "
				 << coefficients->values[1] << ", "
				 << coefficients->values[2] << ", "
				 << coefficients->values[3] << endl;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*Kinect_cloud_filtered);

    i++;
  }

	/*
	 * Source: http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_3:_Cloud_processing_%28advanced%29
	 */
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(Kinect_cloud));
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloudPtr);
	segmentation.setModelType(pcl::SACMODEL_SPHERE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.01);
	segmentation.setRadiusLimits(BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2, BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2);
	segmentation.setOptimizeCoefficients(true);

	pcl::PointIndices inlierIndices;
	segmentation.segment(inlierIndices, *coefficients);*/

	t1 = get_timestamp();
	secs = (t1 - t0) / 1000000.0L;
	std::cout << secs << std::endl;

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
