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
   \author David Silva
   \date   April, 2016
 */

#define _CALIBRATION_PCD_CPP

#include "calibration/calibration_utils.h"
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


// void addCar(const vector<double>& RPY, const vector<double>& translation)
// {
//   tf::Quaternion q;
// 	tf::Vector3 trans;
//
// 	if (translation.empty() && RPY.empty()) //user does not want to translate/rotate clouds and sensors.
// 	{
// 		trans = tf::Vector3( tfScalar(0), tfScalar(0), tfScalar(0) ); // no translation is done
// 		q = tf::createQuaternionFromRPY(0.0, 0.0, 0.0 ); // no rotation
// 	}
// 	else if (translation.empty()) // only rotation given by the user, no translation
// 	{
// 		trans = tf::Vector3( tfScalar(0), tfScalar(0), tfScalar(0) ); // no translation
// 		q = tf::createQuaternionFromRPY( RPY[0], RPY[1], RPY[2] ); // quaternion computation from given angles
// 	}
// 	else // rotation and translation given by the user
// 	{
// 		trans = tf::Vector3( tfScalar(translation[0]), tfScalar(translation[1]), tfScalar(translation[2]) ); // translation given by the user
// 		q = tf::createQuaternionFromRPY( RPY[0], RPY[1], RPY[2] ); // quaternion computation from given angles
// 	}
//
// 	visualization_msgs::Marker marker;
// 	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
// 	marker.header.frame_id = "/my_frame3";
// 	marker.header.stamp = ros::Time();
// 	// Set the namespace and id for this marker.  This serves to create a unique ID
// 	// Any marker sent with the same namespace and id will overwrite the old one
// 	marker.ns = "ATLASCAR1";
// 	marker.id = 0;
// 	// Set the marker type
// 	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
// 	// Set the marker action.  Options are ADD and DELETE
// 	marker.action = visualization_msgs::Marker::ADD;
// 	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// 	marker.pose.position.x = trans[0];
// 	marker.pose.position.y = trans[1];
// 	marker.pose.position.z = trans[2];
// 	marker.pose.orientation.x = q[0];
// 	marker.pose.orientation.y = q[1];
// 	marker.pose.orientation.z = q[2];//0.8836;
// 	marker.pose.orientation.w = q[3];//0.466;
// 	// Set the scale of the marker -- 1x1x1 here means 1m on a side
// 	marker.scale.x = 0.01;
// 	marker.scale.y = 0.01;
// 	marker.scale.z = 0.01;
// 	// Set the color
// 	marker.color.a = 1.0; // Don't forget to set the alpha!
// 	marker.color.r = 0.50;
// 	marker.color.g = 0.50;
// 	marker.color.b = 0.50;
// 	// Publish the marker
// 	marker.mesh_resource = "package://calibration/../STL/ford_escort_atlascar1.stl";
// 	car_pub.publish( marker );
// }
//
// void createDirectory ( )
// {
// 	time_t rawtime;
// 	struct tm * timeinfo;
// 	char buffer[80];
//
// 	time (&rawtime);
// 	timeinfo = localtime(&rawtime);
//
// 	strftime(buffer,80,"%d-%m-%Y_%I-%M-%S",timeinfo);
// 	std::string date(buffer);
//
// 	std::cout << date;
//
//   namespace fs = boost::filesystem;
//   fs::path pkg_path = ros::package::getPath("calibration"); // get calibration package path
//   string path = pkg_path.parent_path().string(); // get parent path and convert to string
//
// 	file_path=path + "/calib_results_pcd/" + date; // complete filepath where results will be saved
// 	const char *file_path_dir = file_path.c_str(); // converts to char*
//
//   // Create directory
// 	const int dir_err = mkdir(file_path_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
// 	if (-1 == dir_err)
// 	{
// 		printf("Error creating directory!n");
// 		exit(1);
// 	}
//
//   // Add "/" so we can save results
// 	file_path = file_path + "/";
// }
//
// /**
//    @brief Write a file with the resulting calibration for lasers
//    @param[in] transformation geometric transformation matrix
//    @param[in] name name and extension of the file
//    @return void
//  */
// void writeFile( pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation,const char* filepath)
// {
// 	FILE* pFile = fopen(filepath, "a");
// 	fprintf(pFile, "%F;%F;%F;%F\n ", transformation(0,0),transformation(0,1),transformation(0,2),transformation(0,3));
// 	fprintf(pFile, "%F;%F;%F;%F\n ", transformation(1,0),transformation(1,1),transformation(1,2),transformation(1,3));
// 	fprintf(pFile, "%F;%F;%F;%F\n ", transformation(2,0),transformation(2,1),transformation(2,2),transformation(2,3));
// 	fclose(pFile);
// }
//
// /**
//    @brief Write a file with the resulting calibration for cameras
//    @param[in] geometric transformation matrix
//    @param[in] name name and extension of the file
//    @return void
//  */
// void writeFileCamera( cv::Mat transformation, const char* transformation_name, string filepath)
// {
// 	cv::FileStorage file(filepath, cv::FileStorage::APPEND);
// 	// Write to file
// 	file << transformation_name << transformation;
// 	file.release();
// }
//
// /**
//    @brief Transformation estimation between sensor pairs
//    @param[out] laser geometric transformation of the calibrated sensor
//    @param[in] target_laserCloud point cloud from the reference sensor
//    @param[in] laserCloud point cloud from the source sensor
//    @param[in] laserNames name of the sensor pairs
//    @return void
//  */
// void estimateTransformation(geometry_msgs::Pose & laser,pcl::PointCloud<pcl::PointXYZ> target_laserCloud, pcl::PointCloud<pcl::PointXYZ> & laserCloud, const char* laserNames)
// {
// 	//Eigen::Matrix4d transformation of laser lms151 to ldmrs
// 	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> TESVD;
// 	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation;
// 	TESVD.estimateRigidTransformation (laserCloud,target_laserCloud,transformation);
// 	cout<<transformation<<endl;
//
// 	MatrixXd Trans(4,4);
// 	Trans<<transformation(0,0), transformation(0,1), transformation(0,2), transformation(0,3),
// 	        transformation(1,0), transformation(1,1), transformation(1,2), transformation(1,3),
// 	        transformation(2,0), transformation(2,1), transformation(2,2), transformation(2,3),
// 	        transformation(3,0), transformation(3,1), transformation(3,2), transformation(3,3);
//
// 	for(int i=0; i<laserCloud.points.size(); i++)
// 	{
// 		Vector4d Point_in(laserCloud.points[i].x,laserCloud.points[i].y,laserCloud.points[i].z,1), Point_out;
//
// 		Point_out=Trans*Point_in;
// 		laserCloud.points[i].x = Point_out(0);
// 		laserCloud.points[i].y = Point_out(1);
// 		laserCloud.points[i].z = Point_out(2);
// 	}
//
// 	string name;
//
// 	if(laserNames=="lms1_ldmrs")
// 		name="lms1_ldmrs_calib.txt";
//
// 	else if(laserNames=="lms1_sr")
// 		name="lms1_sr_calib.txt";
//
// 	else if(laserNames=="lms1_stereo")
// 		name="lms1_stereo_calib_.txt";
//
// 	else if(laserNames=="lms1_lms2")
// 		name="lms1_lms2_calib.txt";
//
// 	else if (laserNames=="lms1_camera")
// 	{
// 		name="lms1_camera_calib.txt";
// 		// Rotation matrix around the X axis so the pose represents the Z axis
// 		double alpha = -M_PI/2;
// 		MatrixXd R_y(4,4);
// 		R_y << cos(alpha), 0, sin(alpha), 0,
// 		        0,        1,     0,      0,
// 		        -sin(alpha), 0, cos(alpha), 0,
// 		        0,           0,     0,      1;
// 		Trans=Trans*R_y;
// 	}
//
// 	/* Visualization of camera position and orientation
// 	   Convert the opencv matrices to tf compatible ones */
// 	tf::Quaternion q;
//
// 	tf::Matrix3x3 T;
// 	T.setValue(Trans(0,0),Trans(0,1),Trans(0,2),
// 	           Trans(1,0),Trans(1,1),Trans(1,2),
// 	           Trans(2,0),Trans(2,1),Trans(2,2));
//
// 	const char* FilePath = (file_path + name).c_str();
//
// 	writeFile(transformation, FilePath);
//
// 	T.getRotation(q);
//
// 	std::cout << "Quaternion: " << q[0] << " "
// 	<< q[1] << " "
// 	<< q[2] << " "
// 	<< q[3] << endl;
//
//
// 	laser.position.x=Trans(0,3);
// 	laser.position.y=Trans(1,3);
// 	laser.position.z=Trans(2,3);
// 	laser.orientation.x=q[0];
// 	laser.orientation.y=q[1];
// 	laser.orientation.z=q[2];
// 	laser.orientation.w=q[3];
// }
//
// int estimateTransformationCamera(geometry_msgs::Pose & camera, vector<cv::Point3f> objectPoints,
// 	vector<cv::Point2f> imagePoints, const char* name, const bool draw, const bool ransac)
// {
// 	//read calibration paraneters
// 	string a="/src/mystereocalib.yml";
// 	string path = ros::package::getPath("calibration");
// 	path=path+a;
// 	cv::FileStorage fs(path, cv::FileStorage::READ);
// 	if(!fs.isOpened())
// 	{
// 		cout<<"failed to open document"<<endl;
// 		return -1;
// 	}
//
// 	// Matrices to store intrinsic parameters and distortion coefficients
// 	cv::Mat intrinsic_matrix, distortion_coeffs;
//
// 	fs["CM1"] >> intrinsic_matrix;
// 	fs["D1"] >> distortion_coeffs;
//
// 	// DEBUGGING =================================================================
// 	// cout << intrinsic_matrix << endl;
// 	// cout << distortion_coeffs << endl;
// 	//
// 	// cout << objectPoints << endl;
// 	// cout << imagePoints << endl;
// 	// DEBUGGING =================================================================
//
// 	string method;
//
// 	cv::Mat rotation_vector(1,3,cv::DataType<double>::type);
// 	cv::Mat translation_vector(1,3,cv::DataType<double>::type);
//
// 	//rotation_vector = (cv::Mat_<double>(1,3) << 0, 0, 0); // DEBUGGING - solvePnP extrinsicGuess
// 	//translation_vector = (cv::Mat_<double>(1,3) << -1, 0, 2); // DEBUGGING - solvePnP extrinsicGuess
//
// 	/* SolvePnP brings points from the model coordinate system to the camera
// 	         coordinate system based on an array of points in the model coordinate space
// 	         (objectPoints) and an array of points in the camera coordinate image system
// 	         (imagePoints) */
// 	if (ransac)
// 	{
// 	solvePnPRansac(objectPoints, imagePoints,
// 		intrinsic_matrix,
// 		cv::noArray(),
// 		rotation_vector,
// 		translation_vector,
// 		false,
// 		100);
// 		method = "solvePnPRansac_";
// 	}
// 	else
// 	{
// 		solvePnP(objectPoints, imagePoints,
// 						intrinsic_matrix,
// 						cv::noArray(),
// 						//distortion_coeffs,
// 						rotation_vector,
// 						translation_vector,
// 						false,
// 						CV_ITERATIVE);
// 		method = "solvePnP_";
// 	}
// 	// No distortion coefficients are given because imagePoints are already undistorted
//
// 	// Project objectPoints to the image to check if solvePnP results are good ===
// 	cv::Mat test_image = cv::imread("img.jpg", CV_LOAD_IMAGE_COLOR);
// 	vector<cv::Point2f> reprojectPoints;
//
// 	cv::projectPoints(objectPoints, rotation_vector, translation_vector,
// 	                  intrinsic_matrix,
// 	                  distortion_coeffs,
// 	                  reprojectPoints);
//
// 	//std::cout << reprojectPoints << std::endl; // DEBUGGING
//
// 	// SolvePnP re-projection error calculation
// 	float sum = 0.;
// 	sum = cv::norm(reprojectPoints, imagePoints);
// 	cout << "SolvePnP re-projection error = " << sum << endl;
//
// 	// Draw imagePoints and projectPoints on an image ans save it
// 	int myradius=5;
// 	for (int i=0; i<reprojectPoints.size(); i++)
// 	{
//     cv::line(test_image, cv::Point(imagePoints[i].x - 5, imagePoints[i].y), cv::Point(imagePoints[i].x + 5, imagePoints[i].y), cv::Scalar(0,255,0), 2);  //crosshair horizontal
//     cv::line(test_image, cv::Point(imagePoints[i].x, imagePoints[i].y - 5), cv::Point(imagePoints[i].x, imagePoints[i].y + 5), cv::Scalar(0,255,0), 2);  //crosshair vertical
// 		//cv::circle(test_image, cv::Point(imagePoints[i].x, imagePoints[i].y), myradius, cv::Scalar(0,255,0),-1,8,0); // Green original points
// 	}
// 	imwrite( file_path + method + "imagePoints.jpg", test_image );
// 	// Done with projected points ================================================
//   for (int i=0; i<reprojectPoints.size(); i++)
// 	{
//   cv::circle(test_image, cv::Point(reprojectPoints[i].x, reprojectPoints[i].y), myradius, cv::Scalar(0,0,255),-1,8,0); // Red reprojected points
//   }
//   imwrite( file_path + method + "projectPoints_imagePoints.jpg", test_image );
//
// 	cv::Mat R(3,3,cv::DataType<double>::type);
// 	cv::Rodrigues(rotation_vector, R); // transforms rotation_vector in rotation matrix R (3x3)
//
// 	cv::Mat C_M(4,4,R.type()); // transformation matrix (4x4)
//
// 	C_M(cv::Range(0,3), cv::Range(0,3)) = R * 1; // copies R into T
// 	C_M(cv::Range(0,3), cv::Range(3,4)) = translation_vector * 1; // copies inv_translation_vector into T
// 	double *p = C_M.ptr<double>(3);
// 	p[0]=p[1]=p[2]=0; p[3]=1;
//
// 	cout << "SolvePnP Camera to LMS = " << C_M << endl; // writes camera to LMS transformation to terminal
//
// 	/* Invert transformation to get transformation from camera to model */
// 	R=R.t(); //For a rotation matrix it's transpose is equal to it's inverse
// 	cv::Mat inv_translation_vector;
// 	inv_translation_vector = -R * translation_vector; // translation_vector inverse
//
//
// 	cv::Mat T(4,4,R.type()); //Transformation matrix (4x4)
// 	T(cv::Range(0,3), cv::Range(0,3)) = R * 1; // copies R into T
// 	T(cv::Range(0,3), cv::Range(3,4)) = inv_translation_vector * 1; // copies inv_translation_vector into T
//
// 	p = T.ptr<double>(3);
// 	p[0]=p[1]=p[2]=0; p[3]=1;
//
// 	cout << "SolvePnP LMS to Camera = " << T << endl; // writes LMS to camera transformation to terminal
//
// 	// Saving transformations on files
// 	string FilePath = file_path + method + name + ".txt";
// 	writeFileCamera(C_M, "C_M", FilePath);
// 	writeFileCamera(T, "M_C", FilePath);
//
// 	/* Visualization of camera position and orientation
// 	         Convert the opensv matrices to tf compatible ones */
// 	// Rotation matrices around the X axis so the pose represents the Z axis
// 	if (draw)
// 	{
// 		double alpha = -M_PI/2;
// 		cv::Mat R_y = (cv::Mat_<double>(4, 4) <<
// 		               cos(alpha), 0, sin(alpha), 0,
// 		               0,          1,     0,       0,
// 		               -sin(alpha), 0, cos(alpha), 0,
// 		               0,           0,     0,      1);
// 		T=T*R_y;
//
// 		tf::Matrix3x3 rot;
// 		rot[0][0] = T.at<double>(0,0);
// 		rot[0][1] = T.at<double>(0,1);
// 		rot[0][2] = T.at<double>(0,2);
// 		rot[1][0] = T.at<double>(1,0);
// 		rot[1][1] = T.at<double>(1,1);
// 		rot[1][2] = T.at<double>(1,2);
// 		rot[2][0] = T.at<double>(2,0);
// 		rot[2][1] = T.at<double>(2,1);
// 		rot[2][2] = T.at<double>(2,2);
// 		tf::Vector3 trans(T.at<double>(0,3),
// 		                  T.at<double>(1,3),
// 		                  T.at<double>(2,3));
//
// 		tf::Quaternion q;
// 		rot.getRotation(q);
// 		camera.position.x=trans[0];
// 		camera.position.y=trans[1];
// 		camera.position.z=trans[2];
// 		camera.orientation.x=q[0];
// 		camera.orientation.y=q[1];
// 		camera.orientation.z=q[2];
// 		camera.orientation.w=q[3];
// 	}
// 	return 0;
// }


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
	//markers_pub = n.advertise<visualization_msgs::Marker>( "/markers3", 10000);

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

  namespace fs = boost::filesystem;
  fs::path pkg_path = ros::package::getPath("calibration"); // get calibration package path
  string pcd_path = pkg_path.parent_path().string(); // get parent path and convert to string
  pcd_path = pcd_path + "/PCDs/";

	FILE* pFile = fopen(FilePath_obj_img, "w");
	fprintf(pFile, "ObjectPoints (m)\tImagePoints (pixel)\n");
	fprintf(pFile, "x;y;z\tx;y;r\n");
	fclose(pFile);


	cout<<"Start calibration"<<endl;

	cout<<"Press Enter to Continue";
	cin.ignore();


	if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path + "lms1.pcd", *Lms1PointCloud) == -1)    //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path + "lms2.pcd", *Lms2PointCloud) == -1)    //* load the file
	{
	 	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	 	return (-1);
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path + "ldmrs.pcd", *LdmrPointCloud) == -1)    //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	//if (pcl::io::loadPCDFile<pcl::PointXYZ> ("stereo.pcd", *Camera1Cloud) == -1) //* load the file
	//{
	//  PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	//  return (-1);
	//}
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path + "singleCamCloud.pcd", *SingleCamCloud) == -1)          //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path + "singleCamCloudPnP.pcd", *SingleCamCloudPnP) == -1)          //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}

	lms1PointCloud.points=(Lms1PointCloud->points);
	lms2PointCloud.points=Lms2PointCloud->points;
	ldmrPointCloud.points=LdmrPointCloud->points;
	//camera1Cloud.points=Camera1Cloud->points;
	singleCamCloud.points=SingleCamCloud->points;
	singleCamCloudPnP.points=SingleCamCloudPnP->points;

	for (int i=0; i<singleCamCloudPnP.points.size(); i++)
	{
		imagePoints.push_back(cv::Point2f(singleCamCloudPnP.points[i].x, singleCamCloudPnP.points[i].y));
		objectPoints.push_back(cv::Point3f(lms1PointCloud.points[i].x, lms1PointCloud.points[i].y, lms1PointCloud.points[i].z));
	}

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
  estimateTransformationCamera(singleCamPnP, objectPoints, imagePoints, "lms1_camera_calib", false, false); // Transformation estimation with solvePnP
  estimateTransformationCamera(singleCamPnPRansac, objectPoints, imagePoints, "lms1_camera_calib", true, true); // Transformation estimation with solvePnPRansac;

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
	clouds.push_back(singleCamCloudPnP);
	clouds.push_back(singleCamCloudPnP);

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
	RPY.push_back(M_PI/2); // X-rotation
	RPY.push_back(0.0); // Y-rotation
	RPY.push_back(M_PI); // Z-rotation

	vector<double> translation;
	translation.push_back(-4.387/2+ 0.05);
	translation.push_back(-1.702/2 + 0.05 );
	translation.push_back(-0.46);

	visualization_msgs::Marker atlascar = addCar(RPY, translation);
  car_pub.publish( atlascar );

	RPY.clear();
	RPY.push_back(0.0);
	RPY.push_back(0.0);
	RPY.push_back(55 * M_PI/180);

	visualization_msgs::MarkerArray targets_markers;
	targets_markers.markers = createTargetMarkers(clouds, lasers, RPY);
	markers_pub.publish(targets_markers);

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


// vector<float> gridEuclideanDistance ( const pcl::PointCloud<pcl::PointXYZ>& p1)
// {
//   float eu_dist, x_dist, y_dist, z_dist;
//   vector<float> eu_dist_vector;
//   // Euclidean distance calculation between points p1[i] and p1[i+1]
//   for (int i=0; i<p1.points.size()-1; i++)
//   {
//     x_dist = p1[i].x - p1[i+1].x;
//     y_dist = p1[i].y - p1[i+1].y;
//     z_dist = p1[i].z - p1[i+1].z;
//     eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );
//     eu_dist_vector.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances
//   }
//   return eu_dist_vector;
// }
//
// vector<float> pointCloudEuclideanDistance ( const pcl::PointCloud<pcl::PointXYZ>& p1, const pcl::PointCloud<pcl::PointXYZ>& p2)
// {
//   float eu_dist, x_dist, y_dist, z_dist;
//   vector<float> eu_dist_vector;
//   // Euclidean distance calculation between correspondent points of p1 and p2
//   for (int i=0; i<p1.points.size(); i++)
//   {
//     x_dist = p1[i].x - p2[i].x;
//     y_dist = p1[i].y - p2[i].y;
//     z_dist = p1[i].z - p2[i].z;
//     eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );
//     eu_dist_vector.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances
//   }
//   return eu_dist_vector;
// }
//
// float vectorMean ( const vector<float>& v )
// {
//   float sum = accumulate(v.begin(), v.end(), 0.0); // sums every element in "v"
//   return sum / v.size(); // mean value
// }
//
// float vectorStdDeviationPopulation ( const vector<float>& v )
// {
//   float m = vectorMean (v); // mean calculation
//
//   float sum_deviation = 0.0;
//   for(int i=0; i<v.size(); ++i)
//     sum_deviation+=(v[i]-m)*(v[i]-m); // adds errors between elements of the "v" and mean value
//
//   return sqrt(sum_deviation / (v.size() )); // population standard deviation
// }
