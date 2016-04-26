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
   \file  calibration.cpp
   \brief Calibration of a set of sensors
   \details Several ball centers are acquired, and used as corresponding points between sensors. Then, using those points is estimated the geometric transformation between sensors pairs.
   \author Marcelo Pereira
   \date   December, 2015
 */

#define _CALIBRATION_CPP_

#include "calibration/calibration.h"
#include "calibration/visualization_rviz_calibration.h"


ros::Publisher markers_pub;
ros::Publisher car_pub;
pcl::PointCloud<pcl::PointXYZ> lms1PointCloud;
pcl::PointCloud<pcl::PointXYZ> lms2PointCloud;
pcl::PointCloud<pcl::PointXYZ> ldmrPointCloud;
pcl::PointCloud<pcl::PointXYZ> swissrangerCloud;
pcl::PointCloud<pcl::PointXYZ> camera1Cloud;
pcl::PointCloud<pcl::PointXYZ> singleCamCloud;
pcl::PointCloud<pcl::PointXYZ> singleCamCloudPnP;
pcl::PointCloud<pcl::PointXYZ>::Ptr Lms1PointCloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr Lms2PointCloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr LdmrPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr Camera1Cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr SingleCamCloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr SingleCamCloudPnP (new pcl::PointCloud<pcl::PointXYZ>);
geometry_msgs::Pose laser_ldmrs;
geometry_msgs::Pose laser_lms151_1;
geometry_msgs::Pose laser_lms151_2;
geometry_msgs::Pose swissranger;
geometry_msgs::Pose stereo;
geometry_msgs::Pose singleCam;
geometry_msgs::Pose singleCamPnP;
geometry_msgs::Pose singleCamPnPRansac;


/**
   @brief Main function of the calibration node
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibration");
	ros::NodeHandle n;

	int num_of_points = 25; // number of points to collect

	markers_pub = n.advertise<visualization_msgs::MarkerArray>( "/markers3", 10000);
	car_pub = n.advertise<visualization_msgs::Marker>("/ATLASCAR1", 1);

	CircleCentroids centroids;
	int count=0, change=0;
	pcl::PointXYZ P[2];
	P[1].x=0;
	P[1].y=0;
	P[1].z=0;

	cout<<"start";
	cin.ignore();

	ldmrPointCloud.clear();
	lms1PointCloud.clear();
	lms2PointCloud.clear();
	swissrangerCloud.clear();
	camera1Cloud.clear();
	singleCamCloud.clear();
	singleCamCloudPnP.clear();

  laser_lms151_1.position.x=0.0;
  laser_lms151_1.position.y=0.0;
  laser_lms151_1.position.z=0.0;
  laser_lms151_1.orientation.x=0.0;
  laser_lms151_1.orientation.y=0.0;
  laser_lms151_1.orientation.z=0.0;
  laser_lms151_1.orientation.w=1.0;

	vector<cv::Point3f> objectPoints;
	vector<cv::Point2f> imagePoints;

	createDirectory( );

	const char* filename_obj_img = "singleCam_obj_img_points.txt"; // file filename that stores object points and image points
	string tmp_str = file_path + filename_obj_img;
	char* FilePath_obj_img = new char [tmp_str.length() + 1];
	strcpy (FilePath_obj_img, tmp_str.c_str());

	FILE* pFile = fopen(FilePath_obj_img, "w");
	fprintf(pFile, "ObjectPoints (m)\tImagePoints (pixel)\n");
	fprintf(pFile, "x;y;z\tx;y;r\n");
	fclose(pFile);

	cout<<"Start calibration"<<endl;

	// =========================================================================
	// Comment below read data from file
	// =========================================================================

	while(count < num_of_points)
	{
		//cout << "calibration: test\n" << "lms1 - " << centroids.lms1Centroid.point.x << " | lms2 - " <<  centroids.lms2Centroid.point.x << endl; // debug
		//cout<<"cycle start"<<endl;
		//cout << "(" << centroids.singleCamCentroid.point.x << "," << centroids.singleCamCentroid.point.y << "," <<  centroids.singleCamCentroid.point.z << ")" << endl;
		if(centroids.lms1Centroid.point.x != 0 && centroids.lms2Centroid.point.x != 0
		   && centroids.ldmrsCentroid.point.x != 0 && centroids.singleCamCentroid.point.z != 0
			 && centroids.singleCamCentroidPnP.point.z != 0)        //&& centroids.cam1Centroid.point.z!=0)
		{
			P[0].x=centroids.lms1Centroid.point.x;
			P[0].y=centroids.lms1Centroid.point.y;
			P[0].z=centroids.lms1Centroid.point.z;

			double dist;
			dist=sqrt(pow((P[0].x-P[1].x),2) + pow((P[0].y-P[1].y),2));
			double time_diference;
			time_diference = centroids.lms2Centroid.header.stamp.nsec - centroids.lms1Centroid.header.stamp.nsec;
			cout<<"time = "<<time_diference<<endl;
			cout<<"dist = "<<dist<<endl;
			if(dist>0.10)
			{
				double diff_dist_mean=0;
				if(count>0)
				{
					double d_ldmrs, d_lms1, d_lms2, d_sr, d_stereo;
					// calculation of the distance between the corrent center of the ball detected and the previous
					d_ldmrs=sqrt(pow((ldmrPointCloud.points[count-1].x-centroids.ldmrsCentroid.point.x),2)+pow((ldmrPointCloud.points[count-1].y-centroids.ldmrsCentroid.point.y),2)+pow((ldmrPointCloud.points[count-1].z-centroids.ldmrsCentroid.point.z),2));
					d_lms1=sqrt(pow((lms1PointCloud.points[count-1].x-centroids.lms1Centroid.point.x),2)+pow((lms1PointCloud.points[count-1].y-centroids.lms1Centroid.point.y),2)+pow((lms1PointCloud.points[count-1].z-centroids.lms1Centroid.point.z),2));
					d_lms2=sqrt(pow((lms2PointCloud.points[count-1].x-centroids.lms2Centroid.point.x),2)+pow((lms2PointCloud.points[count-1].y-centroids.lms2Centroid.point.y),2)+pow((lms2PointCloud.points[count-1].z-centroids.lms2Centroid.point.z),2));
					// d_sr=sqrt(pow((swissrangerCloud.points[count-1].x-centroids.swissrangerCentroid.point.x),2)+pow((swissrangerCloud.points[count-1].y-centroids.swissrangerCentroid.point.y),2)+pow((swissrangerCloud.points[count-1].z-centroids.swissrangerCentroid.point.z),2));
					// d_stereo=sqrt(pow((camera1Cloud.points[count-1].x-centroids.cam1Centroid.point.x),2)+pow((camera1Cloud.points[count-1].y-centroids.cam1Centroid.point.y),2)+pow((camera1Cloud.points[count-1].z-centroids.cam1Centroid.point.z),2));
					int number_of_sensors = 2;         //number of sensors to be calibrated (without counting the reference sensor)
					// diff_dist_mean=(pow((d_lms1-d_ldmrs),2)+pow((d_lms1-d_stereo),2)+pow((d_lms1-d_lms2),2))/number_of_sensors;
					// diff_dist_mean=(pow((d_lms1-d_lms2),2))/number_of_sensors; //teste com 2 LMS151
					diff_dist_mean=(pow((d_lms1-d_ldmrs),2)+pow((d_lms1-d_lms2),2))/number_of_sensors;
				}
				std::cout << "diff_dist_mean = " << diff_dist_mean << std::endl;
				if(diff_dist_mean<0.15)    // Valid point     // ?? Why less and not greater?
				{
					pcl::PointXYZ p;
					p.x=centroids.lms1Centroid.point.x;
					p.y=centroids.lms1Centroid.point.y;
					p.z=centroids.lms1Centroid.point.z;
					lms1PointCloud.push_back(p);

					objectPoints.push_back(cv::Point3f(p.x, p.y, p.z));
					cout << "objectPoint = " << p << endl; // prints current objectPoint


					pFile = fopen(FilePath_obj_img, "a");
					fprintf(pFile, "%F;%F;%F\t", p.x, p.y, p.z);

					p.x=centroids.lms2Centroid.point.x;
					p.y=centroids.lms2Centroid.point.y;
					p.z=centroids.lms2Centroid.point.z;
					lms2PointCloud.push_back(p);

					p.x=centroids.ldmrsCentroid.point.x;
					p.y=centroids.ldmrsCentroid.point.y;
					p.z=centroids.ldmrsCentroid.point.z;
					ldmrPointCloud.push_back(p);

					// p.x=centroids.swissrangerCentroid.point.x;
					// p.y=centroids.swissrangerCentroid.point.y;
					// p.z=centroids.swissrangerCentroid.point.z;
					// swissrangerCloud.push_back(p);
					//
					// p.x=centroids.cam1Centroid.point.x;
					// p.y=centroids.cam1Centroid.point.y;
					// p.z=centroids.cam1Centroid.point.z;
					// camera1Cloud.push_back(p);

					// PointCLoud used on radius based method
					p.x=centroids.singleCamCentroid.point.x;
					p.y=centroids.singleCamCentroid.point.y;
					p.z=centroids.singleCamCentroid.point.z;
					singleCamCloud.push_back(p);
					cout << "singleCamCloud = " << p << endl; // prints current Camera point

					// Image points used on solvePnP based method
					p.x = centroids.singleCamCentroidPnP.point.x;
					p.y = centroids.singleCamCentroidPnP.point.y;
					p.z = centroids.singleCamCentroidPnP.point.z;
					imagePoints.push_back(cv::Point2f(p.x, p.y));
					singleCamCloudPnP.push_back(p); // Point cloud with imagePoints so I can save to PCD file
					cout << "imagePoint = " << p << endl; // prints current imagePoint

					// save imagePoints to file
					fprintf(pFile, "%F;%F;%F\n", p.x, p.y, p.z);
					fclose(pFile);

					cout<<"count "<<count+1<<endl;

					cout<<"Press to select another point";
					cin.ignore();
					count++;
					change=0;
				}

				// Storing point clouds in clouds
				vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
				clouds.push_back(lms1PointCloud);
				clouds.push_back(lms2PointCloud);
				clouds.push_back(ldmrPointCloud);
				//clouds.push_back(swissrangerCloud);
				//clouds.push_back(camera1Cloud);
				clouds.push_back(singleCamCloud);
				clouds.push_back(singleCamCloudPnP); // for SolvePnP
				clouds.push_back(singleCamCloudPnP); // for SolvePnPRansac

				// Saving all point clouds to a PCD file
				pcl::io::savePCDFileASCII(file_path + "lms1.pcd", lms1PointCloud);
				pcl::io::savePCDFileASCII(file_path + "lms2.pcd", lms2PointCloud);
				pcl::io::savePCDFileASCII(file_path + "ldmrs.pcd", ldmrPointCloud);
				//pcl::io::savePCDFileASCII("swissranger.pcd", swissrangerCloud);
				//pcl::io::savePCDFileASCII("stereo.pcd", camera1Cloud);
				pcl::io::savePCDFileASCII(file_path + "singleCamCloud.pcd", singleCamCloud);
				pcl::io::savePCDFileASCII(file_path + "singleCamCloudPnP.pcd", singleCamCloudPnP); // imagePoints and radius cloud

				string imgPath = file_path + "img_" + boost::lexical_cast<std::string>(count) + ".jpg";
				cv::Mat img;
				img = cv_bridge::toCvShare(centroids.camImage, "bgr8")->image;
				imwrite( imgPath, img );

				// Sets pose
				vector<geometry_msgs::Pose> lasers;
				lasers.push_back(laser_lms151_1);
				lasers.push_back(laser_lms151_2);
				lasers.push_back(laser_ldmrs);
				//lasers.push_back(swissranger);
				//lasers.push_back(stereo);
				lasers.push_back(singleCam);
				lasers.push_back(singleCamPnP);
				lasers.push_back(singleCamPnPRansac);

				visualization_msgs::MarkerArray targets_markers;
				targets_markers.markers = createTargetMarkers(clouds,lasers);
				markers_pub.publish(targets_markers);
			}

			if(change==0)
			{
				P[1].x=centroids.lms1Centroid.point.x;
				P[1].y=centroids.lms1Centroid.point.y;
				P[1].z=centroids.lms1Centroid.point.z;
				change=1;
			}
		}
		ros::spinOnce();
	}

	cout<<"Press Enter to Continue";
	cin.ignore();

	// What happens below????
	vector<pcl::PointCloud<pcl::PointXYZ> > cloudss;
	cloudss.push_back(lms1PointCloud);
	cloudss.push_back(lms2PointCloud);
	cloudss.push_back(ldmrPointCloud);
	//cloudss.push_back(swissrangerCloud);
	//cloudss.push_back(camera1Cloud);
	cloudss.push_back(singleCamCloud);
	cloudss.push_back(singleCamCloudPnP); // for SolvePnP
	cloudss.push_back(singleCamCloudPnP); // for SolvePnPRansac

	vector<geometry_msgs::Pose> laserss;
	laserss.push_back(laser_lms151_1);
	laserss.push_back(laser_lms151_2);
	laserss.push_back(laser_ldmrs);
	//laserss.push_back(swissranger);
	//laserss.push_back(stereo);
	laserss.push_back(singleCam);
	laserss.push_back(singleCamPnP);
	laserss.push_back(singleCamPnPRansac);

	visualization_msgs::MarkerArray targets_markerss;
	targets_markerss.markers = createTargetMarkers(cloudss,laserss);
	markers_pub.publish(targets_markerss);

	estimateTransformation(laser_lms151_2,lms1PointCloud,lms2PointCloud, "lms1_lms2");
	estimateTransformation(laser_ldmrs,lms1PointCloud,ldmrPointCloud, "lms1_ldmrs");
	// estimateTransformation(swissranger,lmsPointCloud,swissrangerCloud,"lms_sr");
	// estimateTransformation(stereo,lms1PointCloud,camera1Cloud,"lms1_stereo");
	estimateTransformation(singleCam, lms1PointCloud, singleCamCloud, "lms1_camera");
	estimateTransformationCamera(singleCamPnP, objectPoints, imagePoints, "lms1_camera_calib", true, false); // Transformation estimation with solvePnP
	estimateTransformationCamera(singleCamPnPRansac, objectPoints, imagePoints, "lms1_camera_calib", true, true); // Transformation estimation with solvePnPRansac

	cout<<"end"<<endl;

	cout<<"Press Enter to Continue";
	cin.ignore();

	vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
	clouds.push_back(lms1PointCloud);
	clouds.push_back(lms2PointCloud);
	clouds.push_back(ldmrPointCloud);
	//clouds.push_back(swissrangerCloud);
	//clouds.push_back(camera1Cloud);
	clouds.push_back(singleCamCloud);
	clouds.push_back(singleCamCloudPnP); // for SolvePnP
	clouds.push_back(singleCamCloudPnP); // for SolvePnPRansac

	// Sets pose according to estimated transformations
	vector<geometry_msgs::Pose> lasers;
	lasers.push_back(laser_lms151_1);
	lasers.push_back(laser_lms151_2);
	lasers.push_back(laser_ldmrs);
	//lasers.push_back(swissranger);
	//lasers.push_back(stereo);
	lasers.push_back(singleCam);
	lasers.push_back(singleCamPnP);
	lasers.push_back(singleCamPnPRansac);

	vector<double> RPY;
	// ATLASCAR model rotations
	RPY.push_back(M_PI/2); // X-rotation
	RPY.push_back(0.0); // Y-rotation
	RPY.push_back(M_PI); // Z-rotation

	vector<double> translation;
	translation.push_back(-4.387/2+ 0.05); // X translation. 4.387 is the car's length
	translation.push_back(-1.702/2 + 0.05 ); // Y translation. 1.702 is the car's width
	translation.push_back(-0.46); // Z translation. 0.46 is the height of the reference LMS sensor

	visualization_msgs::Marker atlascar = addCar(RPY, translation); // builds the marker to publish
  car_pub.publish( atlascar );

	RPY.clear();
	// Clouds and lasers rotations so point cloud is aligned with Rviz grid
	RPY.push_back(0.0);
	RPY.push_back(0.0);
	RPY.push_back(55 * M_PI/180);

	visualization_msgs::MarkerArray targets_markers;
	targets_markers.markers = createTargetMarkers(clouds, lasers, RPY);
	markers_pub.publish(targets_markers);

	/*
		============================================================================
		Analyses of results (mean and standard deviations)
		============================================================================
	*/
	float mean, stdDev;
	vector<float> eu_dist_vector;

	eu_dist_vector = pointCloudEuclideanDistance(lms1PointCloud, lms2PointCloud);
	mean = vectorMean(eu_dist_vector);
	stdDev = vectorStdDeviationPopulation(eu_dist_vector);
	cout << "LMS151 2: Mean = " << mean << " Standard deviation = " << stdDev << endl;

	eu_dist_vector = pointCloudEuclideanDistance(lms1PointCloud, ldmrPointCloud);
	mean = vectorMean(eu_dist_vector);
	stdDev = vectorStdDeviationPopulation(eu_dist_vector);
	cout << "LD-MRS: Mean = " << mean << " Standard deviation = " << stdDev << endl;

	eu_dist_vector = pointCloudEuclideanDistance(lms1PointCloud, singleCamCloud);
	mean = vectorMean(eu_dist_vector);
	stdDev = vectorStdDeviationPopulation(eu_dist_vector);
	cout << "Camera 3D Rigid Transf.: Mean = " << mean << " Standard deviation = " << stdDev << endl;

	cout << "Distance between LMS151 points:" << endl;
	vector<float> grid_eu_dist_vector = gridEuclideanDistance(lms1PointCloud);
	count = 1;
	for (vector<float>::const_iterator i = grid_eu_dist_vector.begin(); i != grid_eu_dist_vector.end(); ++i)
	{
		cout << boost::lexical_cast<std::string>(count) << "-" << boost::lexical_cast<std::string>(count+1) << " " << *i << endl;
		count++;
	}

	vector<float> eu_dist_vectorY;
	float eu_dist, x_dist, y_dist, z_dist;

	x_dist = lms1PointCloud[0].x - lms1PointCloud[9].x;
	y_dist = lms1PointCloud[0].y - lms1PointCloud[9].y;
	z_dist = lms1PointCloud[0].z - lms1PointCloud[9].z;
	eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );
	eu_dist_vectorY.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances
	x_dist = lms1PointCloud[1].x - lms1PointCloud[8].x;
	y_dist = lms1PointCloud[1].y - lms1PointCloud[8].y;
	z_dist = lms1PointCloud[1].z - lms1PointCloud[8].z;
	eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );
	eu_dist_vectorY.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances
	x_dist = lms1PointCloud[2].x - lms1PointCloud[7].x;
	y_dist = lms1PointCloud[2].y - lms1PointCloud[7].y;
	z_dist = lms1PointCloud[2].z - lms1PointCloud[7].z;
	eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );
	eu_dist_vectorY.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances
	x_dist = lms1PointCloud[3].x - lms1PointCloud[6].x;
	y_dist = lms1PointCloud[3].y - lms1PointCloud[6].y;
	z_dist = lms1PointCloud[3].z - lms1PointCloud[6].z;
	eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );
	eu_dist_vectorY.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances
	x_dist = lms1PointCloud[5].x - lms1PointCloud[14].x;
	y_dist = lms1PointCloud[5].y - lms1PointCloud[14].y;
	z_dist = lms1PointCloud[5].z - lms1PointCloud[14].z;
	eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );
	eu_dist_vectorY.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances
	x_dist = lms1PointCloud[6].x - lms1PointCloud[13].x;
	y_dist = lms1PointCloud[6].y - lms1PointCloud[13].y;
	z_dist = lms1PointCloud[6].z - lms1PointCloud[13].z;
	eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );
	eu_dist_vectorY.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances
	x_dist = lms1PointCloud[7].x - lms1PointCloud[12].x;
	y_dist = lms1PointCloud[7].y - lms1PointCloud[12].y;
	z_dist = lms1PointCloud[7].z - lms1PointCloud[12].z;
	eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );
	eu_dist_vectorY.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances
	x_dist = lms1PointCloud[8].x - lms1PointCloud[11].x;
	y_dist = lms1PointCloud[8].y - lms1PointCloud[11].y;
	z_dist = lms1PointCloud[8].z - lms1PointCloud[11].z;
	eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );
	eu_dist_vectorY.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances

	cout << "Vertical distance between LMS151 points:" << endl;
	count = 1;
	for (vector<float>::const_iterator i = eu_dist_vectorY.begin(); i != eu_dist_vectorY.end(); ++i)
	{
		cout << boost::lexical_cast<std::string>(count) << "-" << boost::lexical_cast<std::string>(count+1) << " " << *i << endl;
		count++;
	}
}
