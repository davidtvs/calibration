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
   \author Marcelo Pereira
   \date   December, 2015
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

ros::Publisher markers_pub;
ros::Publisher sphereCenter_pub;
ros::Publisher pointCloud_pub;

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
void sphereDetection(pcl::PointCloud<pcl::PointXYZ> SwissRanger_cloud)
{

	timestamp_t t0, t1;
	double secs;
	t0 = get_timestamp();

	geometry_msgs::PointStamped sphereCenter;
	sphereCenter.point.x = -999;
	sphereCenter.point.y = -999;
	sphereCenter.point.z = -999;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(SwissRanger_cloud));
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr sphereModel(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloudPtr));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(sphereModel);
	// Set the maximum allowed distance to the model.
	ransac.setDistanceThreshold(0.01);
	ransac.computeModel();

	vector<int> inlierIndices;
	ransac.getInliers(inlierIndices);
	cout<<" number od inliers "<<inlierIndices.size()<<endl;

	if(inlierIndices.size()>0)
	{
		// Copy all inliers of the model to another cloud.
		pcl::copyPointCloud<pcl::PointXYZ>(*cloudPtr, inlierIndices, *inlierPoints);

		Eigen::VectorXf sphereCoeffs;
		ransac.getModelCoefficients (sphereCoeffs);

		Eigen::VectorXf sphereCoeffsRefined;
		sphereModel->optimizeModelCoefficients (inlierIndices, sphereCoeffs, sphereCoeffsRefined);
    t1 = get_timestamp();
    secs = (t1 - t0) / 1000000.0L;
    std::cout << secs << std::endl;

		if(sphereCoeffsRefined(3)<BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2 && sphereCoeffsRefined(3)<BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2) // +- 5% of BALL_DIAMETER is admissable
		{
			sphereCenter.point.x = sphereCoeffsRefined(0);
			sphereCenter.point.y = sphereCoeffsRefined(1);
			sphereCenter.point.z = sphereCoeffsRefined(2);
			//writeFile(sphereCoeffsRefined);
			cout<<"z "<<sphereCoeffsRefined(2)<<endl;
		}
		cout<<"radius "<<sphereCoeffsRefined(3)<<endl;

		/*pcl::PointCloud<pcl::PointXYZ> cloud;
		for(int i=0; i<inlierPoints->points.size(); i++)
			cloud.push_back(inlierPoints->points[i]);*/
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
	string sub_node_name;
	n.getParam("sub_node_name", sub_node_name);

	std::cout << sub_node_name << std::endl;

	markers_pub = n.advertise<visualization_msgs::MarkerArray>( "BallDetection", 10000);
	sphereCenter_pub = n.advertise<geometry_msgs::PointStamped>("SphereCentroid",1000);
	pointCloud_pub = n.advertise<sensor_msgs::PointCloud>("Pointcloud",10000);

	swissranger cloud(sub_node_name);

	ros::Rate loop_rate(50);

	while(ros::ok())
	{
		if(cloud.cloud.points.size()>0)
		{
			pcl::PointCloud<pcl::PointXYZ> SwissRanger_cloud;
			pcl::PointXYZ p;
			for(int i=0; i<cloud.cloud.points.size(); i++)
			{
				p.x=cloud.cloud.points[i].z;
				p.y=-cloud.cloud.points[i].y;
				p.z=cloud.cloud.points[i].x;
				SwissRanger_cloud.push_back(p);
			}
			sphereDetection(SwissRanger_cloud);
			cloud.cloud.header.frame_id="/my_frame";
			cloud.cloud.header.stamp= ros::Time::now();
			pointCloud_pub.publish(cloud.cloud);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
