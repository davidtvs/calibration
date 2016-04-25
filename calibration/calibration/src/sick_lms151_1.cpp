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
   \file  sick_lms151_1.cpp
   \brief Algorithm for the ball center detection with one of the Sick LMS151 laser
   \author Marcelo Pereira
   \date   December, 2015
 */

#include <lidar_segmentation/lidar_segmentation.h>
#include <lidar_segmentation/clustering.h>
#include <lidar_segmentation/groundtruth.h>
#include "calibration/common_functions.h"
#include "calibration/sick_lms151_1.h"
#include "calibration/visualization_rviz_lms.h"
#include <cmath>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
#include <numeric>
#include <complex>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include "std_msgs/Float64.h"

//Marker's publisher
ros::Publisher markers_lms_1_pub;
ros::Publisher circleCentroid_pub;
int scan_lms_1_header;
int checkCircle=0;

/**
   @brief Handler for the incoming data
   @param groundtruth_points incoming Laser Points
   @param iteration iteration of the Laser Scan
   @return void
 */

void dataFromFileHandler(vector<PointPtr>& groundtruth_points, int iteration)
{
	//cout << "Scan number: " << iteration << endl;

	vector<PointPtr> groundtruth_points_filtered;

	//Filter the laser points
	filterPoints(groundtruth_points,groundtruth_points_filtered,0.01,50);
	//cout<<"end"<<endl;
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

	vector<ClusterPtr> clusters_nn;
	double threshold_nn = 0.2;
	nnClustering( groundtruth_points_filtered, threshold_nn, clusters_nn);

	vector<ClusterPtr> circle;
	Point sphere;
	find_circle(clusters_nn,circle,sphere);
	//      Vizualize the Segmentation results

	visualization_msgs::MarkerArray targets_markers;

	//Apply transformation on circles in relation to sick_lmrs
	if(sphere.x==0)
	{
		sphere.x =-1000;
		sphere.y =0;
		sphere.z =0;
	}

	targets_markers.markers = createTargetMarkers(clusters_nn,circle,sphere);

	markers_lms_1_pub.publish(targets_markers);

	//cout<<"done all"<<endl;

} //end function

/**
   @brief Find circle on laser data and computation of the sphere centroid
   @param[in] clusters segmented scan from the laser
   @param[out] circleP point coordinates of the circle detected for representation on rviz
   @param[out] sphere coordinates of the sphere centroid
   @return double radius of the detected circle
 */
double find_circle(vector<ClusterPtr> clusters, vector<ClusterPtr>& circleP, Point & sphere)
{
	geometry_msgs::PointStamped centroid;
	int count=0;
	for(int k=0; k<clusters.size(); k++)
	{
		ClusterPtr cluster=clusters[k];
		int segment_init, segment_end;
		for (int i=0; i<cluster->support_points.size(); i++)
		{
			// Apply algorithm from
			/*Fast Line, Arc/Circle and Leg Detection from
			   Laser Scan Data in a Player Driver
			   João Xavier∗ , Marco Pacheco† , Daniel Castro† , António Ruano† and Urbano Nunes*/

			segment_init=0;
			segment_end=cluster->support_points.size()-1;

			// Detect if at least 6 points
			double angle[1000];
			double angle_std;
			double vecA[3],vecB[3];
			int n=0;
			if (segment_end-segment_init>=6)
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

				std::cout << "std = " << std << std::endl;
				std::cout << "m = " << m << std::endl;

				//if (m>90 && m<140 && std < 5)
				if (m>105 && m<140 && std < 5)
				{
					// Circle information computation
					double theta = (m-90)/180*M_PI;
					// use 3D but actually Z = 0 because laserscan is planar
					double middle[3];
					middle[0] = (cluster->support_points[segment_end]->x - cluster->support_points[segment_init]->x)/2;
					middle[1] = (cluster->support_points[segment_end]->y - cluster->support_points[segment_init]->y)/2;
					middle[2] = (cluster->support_points[segment_end]->z - cluster->support_points[segment_init]->z)/2;

					double h = sqrt(pow(middle[0],2) + pow(middle[1],2) + pow(middle[2],2)) * tan(theta);
					double height[3];
					height[1]= sqrt( (middle[0]*middle[0]*h*h) / (middle[0]*middle[0] + middle[1]*middle[1]) );
					height[0]= -middle[1]*height[1]/middle[0];
					height[2]= 0;

					double circle[3], radius;
					circle[0] = cluster->support_points[segment_init]->x + middle[0] - height[0];
					circle[1] = cluster->support_points[segment_init]->y + middle[1] - height[1];
					circle[2] = cluster->support_points[segment_init]->z + middle[2] - height[2];
					//radius = sqrt(pow((circle[0]-cluster->support_points[segment_init]->x),2) + pow((circle[1]-cluster->support_points[segment_init]->y),2) + pow((circle[2]-cluster->support_points[segment_init]->z),2));

					double ma,mb,cx,x1,x2,x3,cy,y1,y2,y3;
					x1=cluster->support_points[segment_init]->x;
					x2=cluster->support_points[round(segment_end/2)]->x;
					x3=cluster->support_points[segment_end]->x;
					y1=cluster->support_points[segment_init]->y;
					y2=cluster->support_points[round(segment_end/2)]->y;
					y3=cluster->support_points[segment_end]->y;

					ma=(y2-y1)/(x2-x1);
					mb=(y3-y2)/(x3-x2);

					cx=(ma*mb*(y1-y3)+mb*(x1+x2)-ma*(x2+x3))/(2*(mb-ma));
					cy=-1/ma*(cx-(x1+x2)/2)+(y1+y2)/2;

					//radius=sqrt(pow((cx-x1),2) + pow((cy-y1),2));

					Point Centroid;
					double R;
					CalculateCircle(cluster,R,Centroid);
					radius=R;
					cout << "Radius = " << radius << endl; // DEBUGGING

					centroid.point.x=Centroid.x;
					centroid.point.y=Centroid.y;

					double ballDiameter = BALL_DIAMETER;

					if(radius > ballDiameter/2 + 0.075 && radius < 0) // invalid radius
					{
						centroid.point.x=0;
						centroid.point.y=0;
						centroid.point.z=-10;
						sphere.x=-10000;
						sphere.y=centroid.point.y;
						sphere.z=centroid.point.z;
						double centre[3];
						centre[0] = Centroid.x;
						centre[1] = Centroid.y;
						centre[2] = 0;
						circlePoints(circleP,radius,centre,20);
					}
					else if (radius <= ballDiameter/2) // valid radius
					{
						centroid.point.z=(sqrt(pow(ballDiameter/2,2)-pow(radius,2)));
						sphere.x=centroid.point.x;
						sphere.y=centroid.point.y;
						sphere.z=centroid.point.z;
						//cout<<"x "<<centroid.point.x<<endl;
						double centre[3];
						centre[0] = Centroid.x;
						centre[1] = Centroid.y;
						centre[2] = centroid.point.z;
						circlePoints(circleP,radius,centre,20);
					}
					else // radius above ball radius but within margin of error
					{
						radius = ballDiameter/2; // because radius is bigger than ball radius the laser is close to the ball's equator
						centroid.point.z=(sqrt(pow(ballDiameter/2,2)-pow(radius,2)));
						sphere.x=centroid.point.x;
						sphere.y=centroid.point.y;
						sphere.z=centroid.point.z;
						//cout<<"x "<<centroid.point.x<<endl;
						double centre[3];
						centre[0] = Centroid.x;
						centre[1] = Centroid.y;
						centre[2] = centroid.point.z;
						circlePoints(circleP,radius,centre,20);
					}

					// OLD VERSION =======================================================
					// if(radius>ballDiameter/2)
					//      radius=ballDiameter/2;
					// centroid.point.z=(sqrt(pow(ballDiameter/2,2)-pow(radius,2)));
					// sphere.x=centroid.point.x;
					// sphere.y=centroid.point.y;
					// sphere.z=centroid.point.z;
					// cout<<"x "<<centroid.point.x<<endl;
					//
					// double centre[3];
					// centre[0] = Centroid.x;
					// centre[1] = Centroid.y;
					// centre[2] = centroid.point.z;
					//
					// //writeFile(centre,"lms_dist_2.txt");
					//
					// circlePoints(circleP,radius,centre,20);
					// OLD VERSION =======================================================


					if(!circleP.empty())
						circleP[count]->centroid=cluster->centroid;
					count++;
					checkCircle=1;
				}
				else
				{
					if(checkCircle==0)
					{
						centroid.point.x=0;
						centroid.point.y=0;
						centroid.point.z=-10;
						sphere.x=-10000;
						sphere.y=centroid.point.y;
						sphere.z=centroid.point.z;
					}
				}
			}
		}
	}
	centroid.header.stamp=ros::Time::now();
	circleCentroid_pub.publish(centroid);
}

/**
   @brief Main function of the sick_lms151_1 node
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sick_lms151");
	ros::NodeHandle n;
	sickLMSscan scan;

	markers_lms_1_pub = n.advertise<visualization_msgs::MarkerArray>( "/markers2", 10000);
	circleCentroid_pub = n.advertise<geometry_msgs::PointStamped>( "/laser1/sphereCentroid", 10000);

	//ros::Rate loop_rate(1);

	while(1)
	{
		C_DataFromFilePtr data_gt;
		//cout<<"size "<<scan.scanLaser1.ranges.size()<<endl;
		vector<PointPtr> points;
		if(scan.scanLaser1.ranges.size()!=0)
		{
			convertDataToXY(scan.scanLaser1, data_gt);

			createPointsFromFile(points, data_gt);
			scan_lms_1_header=data_gt->iteration;

			dataFromFileHandler(points, scan_lms_1_header);
		}

		ros::spinOnce();
		//loop_rate.sleep();
	}
	return 0;
}
