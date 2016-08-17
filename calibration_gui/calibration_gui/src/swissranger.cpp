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
   \file  swissranger.cpp
   \brief Algorithm for the ball center detection with the SwissRanger
   \author Marcelo Pereira, David Silva
   \date   May, 2016
 */

#include "ros/ros.h"
#include <cmath>
#include <algorithm>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include "calibration_gui/swissranger.h"
#include <eigen3/Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/io/pcd_io.h>
#include "calibration_gui/visualization_rviz_swissranger.h"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>


#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/common/transforms.h>

ros::Publisher markers_pub;
ros::Publisher sphereCenter_pub;
ros::Publisher pointCloud_pub;

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
void sphereDetection(pcl::PointCloud<pcl::PointXYZ> SwissRanger_cloud)
{

	ros::Time start = ros::Time::now();

	geometry_msgs::PointStamped sphereCenter;
	sphereCenter.point.x = -999;
	sphereCenter.point.y = -999;
	sphereCenter.point.z = -999;

	/* METHOD #1 ================================================================
	 * Detects the ball up to 2-2.5 meters, very slow

	   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(SwissRanger_cloud));
	   pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	   pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr sphereModel(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloudPtr));
	   pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(sphereModel);
	   // Set the maximum allowed distance to the model.
	   ransac.setDistanceThreshold(0.05);
	   //ransac.setMaxIterations(1000);
	   ransac.computeModel();

	   vector<int> inlierIndices;
	   ransac.getInliers(inlierIndices);
	   cout<<" number od inliers "<<inlierIndices.size()<<endl;

	   if(inlierIndices.indices.size()>0)
	   {
	        // Copy all inliers of the model to another cloud.
	        pcl::copyPointCloud<pcl::PointXYZ>(*cloudPtr, inlierIndices, *inlierPoints);

	        Eigen::VectorXf sphereCoeffs;
	        ransac.getModelCoefficients (sphereCoeffs);

	        Eigen::VectorXf sphereCoeffsRefined;
	        sphereModel->optimizeModelCoefficients (inlierIndices, sphereCoeffs, sphereCoeffsRefined);


	        if(sphereCoeffsRefined(3)<BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2 && sphereCoeffsRefined(3)>BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2)    // +- 5% of BALL_DIAMETER is admissable
	        {
	                sphereCenter.point.x = sphereCoeffsRefined(0);
	                sphereCenter.point.y = sphereCoeffsRefined(1);
	                sphereCenter.point.z = sphereCoeffsRefined(2);
	                //writeFile(sphereCoeffsRefined);
	                cout<<"z "<<sphereCoeffsRefined(2)<<endl;
	        }
	        cout<<"radius "<<sphereCoeffsRefined(3)<<endl;
	   }*/


	/* METHOD #2 ================================================================
	 * Detects the ball up to 1.8-2 meters, slowish
	 * Source: http://www.pointclouds.org/documentation/tutorials/cylinder_segmentation.php#cylinder-segmentation
	 * http://answers.ros.org/question/229784/detecting-spheres-using-ransac-in-pcl/

	   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(SwissRanger_cloud));
	   pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normals;
	   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	   // Estimate point normals
	   normals.setSearchMethod (tree);
	   normals.setInputCloud (cloudPtr);
	   normals.setKSearch (50);
	   normals.compute (*cloud_normals);

	   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentation;
	   segmentation.setInputCloud(cloudPtr);
	   segmentation.setInputNormals (cloud_normals);
	   segmentation.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
	   segmentation.setMethodType(pcl::SAC_RANSAC);
	   segmentation.setDistanceThreshold(0.01);
	   segmentation.setOptimizeCoefficients(true);

	   pcl::PointIndices inlierIndices;
	   segmentation.segment(inlierIndices, *coefficients);

	   if(inlierIndices.indices.size()>0)
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
	   }*/

	/* METHOD #3 ================================================================
	 * Detects the ball up to 2-2.3 meters, very fast
	 * Source: http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_3:_Cloud_processing_%28advanced%29
	 */
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(SwissRanger_cloud));
	   pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	   pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	   segmentation.setInputCloud(cloudPtr);
	   segmentation.setModelType(pcl::SACMODEL_SPHERE);
	   segmentation.setMethodType(pcl::SAC_RANSAC);
	   segmentation.setDistanceThreshold(0.01);
	   segmentation.setOptimizeCoefficients(true);

	   pcl::PointIndices inlierIndices;
	   segmentation.segment(inlierIndices, *coefficients);

	   t1 = get_timestamp();
	   secs = (t1 - t0) / 1000000.0L;
	   //std::cout << secs << std::endl;

	   if(inlierIndices.indices.size()>0)
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
	   }*/

	/* METHOD #4 ================================================================
	 * Detects the ball up to 3 meters, fast. Optimized Method #3
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr SR_cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(SwissRanger_cloud));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_SPHERE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.0070936);
	seg.setProbability(0.99);
	seg.setRadiusLimits (BALL_DIAMETER/2-0.05, BALL_DIAMETER/2+0.05);
	seg.setInputCloud (SR_cloudPtr);
	seg.segment (*inliers, *coefficients);

	ros::Time end = ros::Time::now();

	cout << "Ball detection time filtered: " <<  (end - start).toNSec() * 1e-6 << " msec"  << endl;

	cout << *coefficients << endl;

	if(inliers->indices.size ()>0)
	{

		if (coefficients->values[3]<BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2 && coefficients->values[3]>BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2)
		{
			sphereCenter.point.x = coefficients->values[0];
			sphereCenter.point.y = coefficients->values[1];
			sphereCenter.point.z = coefficients->values[2];


			cout << "Accepted: " << *coefficients << endl;
		}
	}

	sphereCenter.header.stamp = ros::Time::now();
	sphereCenter_pub.publish(sphereCenter);

	pcl::PointXYZ center;
	center.x = sphereCenter.point.x;
	center.y = sphereCenter.point.y;
	center.z = sphereCenter.point.z;

	visualization_msgs::MarkerArray targets_markers;
	targets_markers.markers = createTargetMarkers(center);
	markers_pub.publish(targets_markers);

}

/**
   @brief Main function of the swissranger node
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "swissranger");

	ros::NodeHandle n("~");
	string node_ns = ros::this_node::getNamespace();
	node_ns.erase(0, 2);
	n.getParam("ballDiameter", BALL_DIAMETER);
	cout << "Node namespace:" << node_ns << endl;
	cout << "Ball diameter:" << BALL_DIAMETER << endl;

	markers_pub = n.advertise<visualization_msgs::MarkerArray>( "BallDetection", 10000);
	sphereCenter_pub = n.advertise<geometry_msgs::PointStamped>("SphereCentroid",1000);
	pointCloud_pub = n.advertise<sensor_msgs::PointCloud>("Pointcloud",10000);

	swissranger cloud(node_ns);

	ros::Rate loop_rate(50);

	while(ros::ok())
	{
		if(cloud.cloud.points.size()>0)
		{
			pcl::PointCloud<pcl::PointXYZ> SwissRanger_cloud;
			pcl::PointXYZ p;
			for(int i=0; i<cloud.cloud.points.size(); i++)
			{
				p.x=cloud.cloud.points[i].x;
				p.y=cloud.cloud.points[i].y;
				p.z=cloud.cloud.points[i].z;
				SwissRanger_cloud.push_back(p);
			}
			sphereDetection(SwissRanger_cloud);
			cloud.cloud.header.frame_id = "/my_frame";
			cloud.cloud.header.stamp = ros::Time::now();
			pointCloud_pub.publish(cloud.cloud);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
