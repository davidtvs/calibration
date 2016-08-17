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
   \file  sick_ldmrs.cpp
   \brief Algorithm for the ball center detection with the Sick LD-MRS laser
   \author Marcelo Pereira
   \date   December, 2015
 */

#define _LDMRS_CPP_

#include <lidar_segmentation/lidar_segmentation.h>
#include "calibration_gui/sick_ldmrs.h"
#include "calibration_gui/common_functions.h"
#include <lidar_segmentation/clustering.h>
#include <lidar_segmentation/groundtruth.h>
#include "calibration_gui/visualization_rviz_ldmrs.h"
#include <cmath>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
#include <numeric>
#include <complex>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_circle.h>


//Marker's publisher
ros::Publisher markers_ldmrs_pub;
ros::Publisher sphereCentroid_pub;

geometry_msgs::PointStamped sphereCentroid;
vector <int> scan_ldmrs_header;

/**
   @brief Handler for the incoming data
   @param[in] lidarPoints incoming Laser Points
   @param[in] iterations iteration of the Laser Scan
   @return void
 */
void dataFromFileHandler(vector<MultiScanPtr>& lidarPoints, vector<int> iterations)
{
	vector<LidarClustersPtr> clusters;
	vector<LidarClustersPtr> circlePoints;
	vector<double> radius;
	vector<geometry_msgs::Point> center;
	Point sphere;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	for(int n=0; n<lidarPoints.size(); n++)
	{
		vector<PointPtr> groundtruth_points = lidarPoints[n]->Points;

		int iteration = iterations[n];

		//cout << "Scan number: " << iteration << endl;

		vector<PointPtr> groundtruth_points_filtered;

		//Filter the laser points
		filterPoints(groundtruth_points,groundtruth_points_filtered,0.01,200.);

		//Sort them by label
		vector<PointPtr> groundtruth_points_filtered_sorted  =  groundtruth_points_filtered;
		sort(groundtruth_points_filtered_sorted.begin(),groundtruth_points_filtered_sorted.end(),comparePoints);

		//-------------------------------------------------------------------------------------------------------------------
		//Make GT clusters

		vector<ClusterPtr> clusters_GT;
		convertPointsToCluster(groundtruth_points_filtered_sorted, clusters_GT);
		//Remove GT clusters with less than a certain size
		uint minimum_points = 3;
		vector<PointPtr> groundtruth_small_points_removed = groundtruth_points_filtered;
		removeInvalidPoints(groundtruth_small_points_removed, clusters_GT, minimum_points);

		//      convertPointsToCluster(groundtruth_points_filtered , clusters_GTs);
		LidarClustersPtr cluster (new LidarClusters);
		vector<ClusterPtr> clusters_nn;
		double threshold_nn = 0.20;
		nnClustering( groundtruth_points_filtered, threshold_nn, clusters_nn);

		cluster->Clusters = clusters_nn;
		clusters.push_back(cluster);

		LidarClustersPtr circlePs (new LidarClusters);
		vector<ClusterPtr> circleP;
		double r;
		r=find_circle(clusters_nn,circleP,n);
		int num;
		if(r!=0)
			num++;
		radius.push_back(r);
		circlePs->Clusters = circleP;
		circlePoints.push_back(circlePs);

		//publish circle centroid

		center.push_back(sphereCentroid.point);

		int circlesNumb = 0;
		if(n==3)
		{
			for(int i=0; i<4; i++)
			{
				if(radius[i]>0.001)
					circlesNumb++;
			}
		}

		if(n==3)
		{
			// cout<<"radius 1 "<<radius[0]<<endl;
			// cout<<"radius 2 "<<radius[1]<<endl;
			// cout<<"radius 3 "<<radius[2]<<endl;
			// cout<<"radius 4 "<<radius[3]<<endl;
		}

		if(n==3)
		{
			if(circlesNumb>1)
			{
				calculateSphereCentroid(center, sphereCentroid, radius);
				sphere.x=sphereCentroid.point.x;
				sphere.y=sphereCentroid.point.y;
				sphere.z=sphereCentroid.point.z;
			}
			else
			{
				sphere.x=-100;
				sphere.y=0;
				sphere.z=0;
			}
			sphereCentroid.header.stamp = ros::Time::now();
			sphereCentroid_pub.publish(sphereCentroid);
		}
	}
	vector<ClusterPtr> linePoints;
	//      Vizualize the Segmentation results

	visualization_msgs::MarkerArray targets_markers;
	targets_markers.markers = createTargetMarkers(clusters,circlePoints, sphere,radius);

	markers_ldmrs_pub.publish(targets_markers);

	//cout<<"done all"<<endl;

} //end function

/**
   @brief Calculation of sphere centroid
   @param[in] center coordinates of circle centers from the four layers
   @param[out] sphereCentroid coordinates of the sphere centroid
   @param[in] radius radius of the several circles
   @return void
 */
void calculateSphereCentroid(vector<geometry_msgs::Point> center, geometry_msgs::PointStamped & sphereCentroid, vector<double> radius)
{
	int count=0;
	double d;
	sphereCentroid.point.x=0;
	sphereCentroid.point.y=0;
	sphereCentroid.point.z=0;
	int s0, s1, s2, s3;
	s0=1;
	s1=1;
	s2=1;
	s3=1;

	// cout << "calculateSphereCentroid Radius = " << endl;
	// cout << radius[0] << endl;
	// cout << radius[1] << endl;
	// cout << radius[2] << endl;
	// cout << radius[3] << endl;

	for(int i=0; i<4; i++)
	{
		double ballDiameter = BALL_DIAMETER;
		if(i==0 && radius[0]>0)
		{
			rotatePoints(center[0].x,center[0].y,center[0].z,-1.2*M_PI/180);
			d=pow(ballDiameter/2,2)-pow(radius[0],2);
			if(d<0)
				d=0;
			center[0].z=s0* sqrt(d);
			rotatePoints(center[0].x,center[0].y,center[0].z,1.2*M_PI/180);
			sphereCentroid.point.x+=(center[0].x);
			sphereCentroid.point.y+=(center[0].y);
			sphereCentroid.point.z+=(center[0].z);
			count++;
		}
		if(i==1 && radius[1]>0)
		{
			rotatePoints(center[1].x,center[1].y,center[1].z,-0.4*M_PI/180);
			d=pow(ballDiameter/2,2)-pow(radius[1],2);
			if(d<0)
				d=0;
			center[1].z= s1*sqrt(d);
			rotatePoints(center[1].x,center[1].y,center[1].z,0.4*M_PI/180);
			sphereCentroid.point.x+=(center[1].x);
			sphereCentroid.point.y+=(center[1].y);
			sphereCentroid.point.z+=(center[1].z);
			count++;
		}
		if(i==2 && radius[2]>0)
		{
			rotatePoints(center[2].x,center[2].y,center[2].z,0.4*M_PI/180);
			d=pow(ballDiameter/2,2)-pow(radius[2],2);
			if(d<0)
				d=0;
			center[2].z= s2*sqrt(d);
			rotatePoints(center[2].x,center[2].y,center[2].z,-0.4*M_PI/180);
			sphereCentroid.point.x+=(center[2].x);
			sphereCentroid.point.y+=(center[2].y);
			sphereCentroid.point.z+=(center[2].z);
			count++;
		}
		if(i==3 && radius[3]>0)
		{
			rotatePoints(center[3].x,center[3].y,center[3].z,1.2*M_PI/180);
			d=pow(ballDiameter/2,2)-pow(radius[3],2);
			if(d<0)
				d=0;
			center[3].z= s3*sqrt(d);
			rotatePoints(center[3].x,center[3].y,center[3].z,-1.2*M_PI/180);
			sphereCentroid.point.x+=(center[3].x);
			sphereCentroid.point.y+=(center[3].y);
			sphereCentroid.point.z+=(center[3].z);
			count++;
		}
	}

	sphereCentroid.point.x=sphereCentroid.point.x/count;
	sphereCentroid.point.y=sphereCentroid.point.y/count;
	sphereCentroid.point.z=sphereCentroid.point.z/count;
	double centre[3];
	centre[0]=sphereCentroid.point.x;
	centre[1]=sphereCentroid.point.y;
	centre[2]=sphereCentroid.point.z;

	//writeFile(centre,"ldmrs_dist_2.txt");
}

/**
   @brief Find circle on laser data
   @param[in] clusters segmented scan from the laser
   @param[out] circleP point coordinates of the circle detected for representation on rviz
   @param[in] layer number of the scan
   @return double radius of the detected circle
 */
double find_circle(vector<ClusterPtr> clusters, vector<ClusterPtr>& circleP, int layer)
{
	int count=0;
	double radius=0;
	bool checkCircle = false;
	for(int k=0; k<clusters.size(); k++)
	{
		ClusterPtr cluster=clusters[k];
		int segment_init, segment_end;
		for (int i=0; i<cluster->support_points.size(); i++)
		{

			segment_init=0;
			segment_end=cluster->support_points.size()-1;

			// Detect if at least 6 points
			double angle[1000];
			double angle_std;
			double vecA[3],vecB[3];
			int n=0;
			if (segment_end-segment_init>=5)
			{
				for (int j=segment_init+1; j<segment_end-1; j++)
				{
					// use 3D but actually Z = 0 because laserscan is planar
					vecA[0] = cluster->support_points[segment_init]->x - cluster->support_points[j]->x;
					vecA[1] = cluster->support_points[segment_init]->y - cluster->support_points[j]->y;
					vecA[2] = cluster->support_points[segment_init]->z - cluster->support_points[j]->z;

					vecB[0] = cluster->support_points[segment_end]->x - cluster->support_points[j]->x;
					vecB[1] = cluster->support_points[segment_end]->y - cluster->support_points[j]->y;
					vecB[2] = cluster->support_points[segment_end]->z - cluster->support_points[j]->z;

					angle[n] = acos( inner_product(vecA, vecA+3, vecB, 0.0)
					                 / (sqrt(pow(vecA[0],2) + pow(vecA[1],2) + pow(vecA[2],2)) * sqrt(pow(vecB[0],2) + pow(vecB[1],2) + pow(vecB[2],2))));
					n++;
				}
				// compute average angle and std
				double m=0;
				double std=0;

				for (int j = 0; j < n; j++)
					m += angle[j];

				m = m/n;
				for ( int j = 0; j < n; j++ )
				{
					std += pow((angle[j]-m),2);
				}

				std = sqrt(std/(n-1));

				// conversion to degree
				m = m/M_PI*180;
				std = std/M_PI*180;

				//if (m>90 && m<135 && std < 8.6)
				//if (m>90 && m<145 && std < 12) // ATLASCAR
				if (m>90 && m<145 && std < 8.5)
				{
					// std::cout << "std = " << std << std::endl;
					// std::cout << "m = " << m << std::endl;
					double ma,mb,cx,x1,x2,x3,cy,y1,y2,y3,z1,z2,z3;
					x1=cluster->support_points[segment_init]->x;
					x2=cluster->support_points[round(segment_end/2)]->x;
					x3=cluster->support_points[segment_end]->x;
					y1=cluster->support_points[segment_init]->y;
					y2=cluster->support_points[round(segment_end/2)]->y;
					y3=cluster->support_points[segment_end]->y;
					z1=cluster->support_points[segment_init]->z;
					z2=cluster->support_points[round(segment_end/2)]->z;
					z3=cluster->support_points[segment_end]->z;

					//rotate da points to da plane XY
					double Angle;
					if(layer==0)
						Angle=-1.2*M_PI/180;
					else if(layer==1)
						Angle=-0.4*M_PI/180;
					else if(layer==2)
						Angle=0.4*M_PI/180;
					else if(layer==3)
						Angle=1.2*M_PI/180;

					for(int i=0; i<cluster->support_points.size(); i++)
						rotatePoints(cluster->support_points[i]->x,cluster->support_points[i]->y, cluster->support_points[i]->z, Angle);

					Point centroid;
					double R;
					CalculateCircle(cluster,R,centroid);

					double z=0;
					rotatePoints(centroid.x,centroid.y,z,-Angle);

					for(int i=0; i<cluster->support_points.size(); i++)
						rotatePoints(cluster->support_points[i]->x,cluster->support_points[i]->y, cluster->support_points[i]->z, -Angle);


					rotatePoints(x1,y1, z1, Angle);
					rotatePoints(x2,y2, z2, Angle);
					rotatePoints(x3,y3, z3, Angle);

					ma=(y2-y1)/(x2-x1);
					mb=(y3-y2)/(x3-x2);

					cx=(ma*mb*(y1-y3)+mb*(x1+x2)-ma*(x2+x3))/(2*(mb-ma));
					cy=-1/ma*(cx-(x1+x2)/2)+(y1+y2)/2;

					radius=sqrt(pow((cx-x1),2) + pow((cy-y1),2));
					radius=R;

					rotatePoints(cx,cy,z1,-Angle);
					double circle[3];
					circle[0]=centroid.x;
					circle[1]=centroid.y;
					circle[2]=z;

					double centre[3];
					centre[0] = circle[0];
					centre[1] = circle[1];
					centre[2] = circle[2];

					sphereCentroid.point.x=circle[0];
					sphereCentroid.point.y=circle[1];
					sphereCentroid.point.z=circle[2];

					circlePoints(circleP,radius,centre,20);
					if(!circleP.empty())
						circleP[count]->centroid=cluster->centroid;
					count++;
					checkCircle=true;
				}
				else
				{
					if(checkCircle==false)
					{
						sphereCentroid.point.x=-999;
						sphereCentroid.point.y=-999;
						sphereCentroid.point.z=-999;
					}
				}
			}
		}
	}
	cout<<"R "<<radius<<endl;
	return radius;
}


/**
   @brief Rotate a point
   @param[out] x x coordinate
   @param[out] y y coordinate
   @param[out] z z coordinate
   @param[out] angle angle to rotate
   @return void
 */
void rotatePoints(double& x,double& y, double& z, double angle)
{
	double X=x,Y=y,Z=z;
	x=X*cos(angle*M_PI/180) + Z*sin(angle*M_PI/180);
	z=X*(-sin(angle*M_PI/180)) + Z*cos(angle*M_PI/180);
	y=Y;
}

/**
   @brief Convert data to XYZ
   @param[in] scan laser scan
   @param[out] data_gt converted data
   @param[in] rot rotation of the scan layer in relation to the XY plane
   @return void
 */
void convertDataToXYZ(sensor_msgs::LaserScan scan, vector<C_DataFromFilePtr>& data_gt, double rot)
{
	C_DataFromFilePtr data (new C_DataFromFile);
	int s=scan.ranges.size();
	int l=1;
	double X, Y;
	for(int n=0; n<s; n++)
	{
		double angle, d, x, y, z;
		d=scan.ranges[n]*cos(rot*M_PI/180);
		z=scan.ranges[n]*sin(rot*M_PI/180);
		angle = scan.angle_min + n*scan.angle_increment;

		x=d*cos(angle);
		y=d*sin(angle);

		X+=x;
		Y+=y;
		data->x_valuesf.push_back(x);
		data->y_valuesf.push_back(y);
		data->z_valuesf.push_back(z);
		data->labels.push_back(l);
	}
	data->iteration=s;
	data_gt.push_back(data);
}

/**
   @brief Main function of the sick_ldmrs node
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sick_ldmrs");

	ros::NodeHandle n("~");
	string node_ns = ros::this_node::getNamespace();
	node_ns.erase(0, 2);
	n.getParam("ballDiameter", BALL_DIAMETER);
	cout << "Node namespace:" << node_ns << endl;
	cout << "Ball diameter:" << BALL_DIAMETER << endl;

	sickLDMRSscan scan(node_ns);

	markers_ldmrs_pub = n.advertise<visualization_msgs::MarkerArray>( "BallDetection", 10000);
	sphereCentroid_pub = n.advertise<geometry_msgs::PointStamped>("SphereCentroid",1000);

	ros::Rate loop_rate(50);

	while(ros::ok())
	{
		vector<C_DataFromFilePtr> data_gt;
		//cout<<"size "<<scan.scan0.ranges.size()<<endl;
		vector<MultiScanPtr> lidarPoints;
		vector<PointPtr> points;

		if(scan.scan3.ranges.size()!=0)
		{

			//first scan
			MultiScanPtr scanPoints0 (new MultiScan);
			convertDataToXYZ(scan.scan0, data_gt,-1.2);

			createPointsFromFile(points, data_gt[0] );
			scanPoints0->Points=points;
			lidarPoints.push_back(scanPoints0);
			scan_ldmrs_header.push_back(data_gt[0]->iteration);

			//second scan
			MultiScanPtr scanPoints1 (new MultiScan);
			convertDataToXYZ(scan.scan1, data_gt,-0.4);
			points.clear();

			createPointsFromFile(points, data_gt[1] );
			scanPoints1->Points=points;
			lidarPoints.push_back(scanPoints1);
			scan_ldmrs_header.push_back(data_gt[1]->iteration);

			//third scan
			MultiScanPtr scanPoints2 (new MultiScan);
			convertDataToXYZ(scan.scan2, data_gt,0.4);
			points.clear();

			createPointsFromFile(points, data_gt[2] );
			scanPoints2->Points=points;
			lidarPoints.push_back(scanPoints2);
			scan_ldmrs_header.push_back(data_gt[2]->iteration);

			//fourth scan
			MultiScanPtr scanPoints3 (new MultiScan);
			convertDataToXYZ(scan.scan3, data_gt,1.2);
			points.clear();

			createPointsFromFile(points, data_gt[3] );
			scanPoints3->Points=points;
			lidarPoints.push_back(scanPoints3);
			scan_ldmrs_header.push_back(data_gt[3]->iteration);


			dataFromFileHandler(lidarPoints, scan_ldmrs_header);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
