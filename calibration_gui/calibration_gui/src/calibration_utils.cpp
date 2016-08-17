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
   \file  calibration_utils.cpp
   \brief Common functions for calibration.cpp and calibration_pcd.cpp
   \details Several helper functions that estimate transforms, draw and analyse results for calibration.cpp and calibration_pcd.cpp
   \author David Silva
   \date   April, 2016
 */

#define _CALIBRATION_UTILS_CPP_

#include "calibration_gui/calibration.h"

/**
   @brief Creates a directory to save results.
   The folder name is the current date and time
   @param void
   @return void
 */
void createDirectory ( )
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer,80,"%d-%m-%Y_%H-%M-%S",timeinfo);
	std::string date(buffer);

  namespace fs = boost::filesystem;
  fs::path pkg_path = ros::package::getPath("calibration_gui"); // get calibration package path
  string path = pkg_path.parent_path().string(); // get parent path and convert to string

	file_path=path + "/calib_results/" + date; // complete filepath where results will be saved
	const char *file_path_dir = file_path.c_str(); // converts to char*

  // Create directory
	const int dir_err = mkdir(file_path_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (-1 == dir_err)
	{
		printf("Error creating directory!n");
		exit(1);
	}

	cout << "Data and results saved on: " << file_path << endl;

  // Add "/" so we can save results
	file_path = file_path + "/";
}

/**
   @brief Write a file with the resulting calibration for lasers
   @param[in] transformation geometric transformation matrix
   @param[in] filepath is an absolute filepath with filename and extension
   @return void
 */
void writeFile( const Matrix4f transformation, const string filepath)
{
  const char* FilePath = filepath.c_str();
  ofstream myfile;
  myfile.open (FilePath, ios::out);
  myfile << transformation;
  myfile.close();
}

/**
   @brief Write a file with the resulting calibration for cameras
   @param[in] transformation transformation matrix
   @param[in] transformation_name variable name given to the geometric transformation
   @param[in] filepath is an absolute filepath with filename and extension
   @return void
 */
void writeFileCamera( cv::Mat transformation, const char* transformation_name, const string filepath)
{
	cv::FileStorage file(filepath, cv::FileStorage::APPEND);
	// Write to file
	file << transformation_name << transformation;
	file.release();
}

/**
   @brief Transformation estimation between sensor pairs using the 3D rigid transformation algorithm from PCL
   @param[out] laser geometric transformation of the calibrated sensor
   @param[in] target_laserCloud point cloud from the reference sensor
   @param[out] laserCloud point cloud from the source sensor
   @param[in] targetSensorName name of the reference sensor
   @param[in] sensorName name of the source sensor
   @return void
 */
void estimateTransformation(geometry_msgs::Pose & laser,pcl::PointCloud<pcl::PointXYZ> target_laserCloud,
	pcl::PointCloud<pcl::PointXYZ> & laserCloud, const string targetSensorName, const string sensorName)
{
	//Eigen::Matrix4d transformation of laser lms151 to ldmrs
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation;
	TESVD.estimateRigidTransformation (laserCloud,target_laserCloud,transformation);
	cout<<transformation<<endl;

	Matrix4f Trans;
	Trans<<transformation(0,0), transformation(0,1), transformation(0,2), transformation(0,3),
	        transformation(1,0), transformation(1,1), transformation(1,2), transformation(1,3),
	        transformation(2,0), transformation(2,1), transformation(2,2), transformation(2,3),
					transformation(3,0), transformation(3,1), transformation(3,2), transformation(3,3);

	for(int i=0; i<laserCloud.points.size(); i++)
	{
		Vector4f Point_in(laserCloud.points[i].x,laserCloud.points[i].y,laserCloud.points[i].z, 1), Point_out;

		Point_out=Trans*Point_in;
		laserCloud.points[i].x = Point_out(0);
		laserCloud.points[i].y = Point_out(1);
		laserCloud.points[i].z = Point_out(2);
	}


	/* Visualization of camera position and orientation
	   Convert the opencv matrices to tf compatible ones */
	tf::Quaternion q;

	tf::Matrix3x3 T;
	T.setValue(Trans(0,0),Trans(0,1),Trans(0,2),
	           Trans(1,0),Trans(1,1),Trans(1,2),
	           Trans(2,0),Trans(2,1),Trans(2,2));

	T.getRotation(q);


		std::cout << "Quaternion " << targetSensorName + "_" + sensorName << ": " << q[0] << " "
		<< q[1] << " "
		<< q[2] << " "
		<< q[3] << endl;

	laser.position.x=Trans(0,3);
	laser.position.y=Trans(1,3);
	laser.position.z=Trans(2,3);
	laser.orientation.x=q[0];
	laser.orientation.y=q[1];
	laser.orientation.z=q[2];
	laser.orientation.w=q[3];

	string name = targetSensorName + "_" + sensorName + "_calib.txt";

	string FilePath = file_path + name;

	writeFile(Trans, FilePath);
}

/**
   @brief Transformation estimation between sensor pairs using the extrinsic calibration algorithm from OpenCV
   @param[out] camera geometric transformation of the calibrated sensor
   @param[in] targetCloud point cloud from the reference sensor
   @param[out] cameraPnPCloud ball centers point cloud in the camera image plane
   @param[in] targetSensorName name of the reference sensor
   @param[in] cameraName name of the camera to be calibrated
   @param[in] projImage image acquired from the camera where \p targetCloud is going to be projected
   @param[in] draw if true the projected points are drawn on the image
   @param[in] ransac if true the RANSAC extrinsic calibration algorithm is used
   @return 0 on success
 */
int estimateTransformationCamera(geometry_msgs::Pose & camera, pcl::PointCloud<pcl::PointXYZ> targetCloud, pcl::PointCloud<pcl::PointXYZ> cameraPnPCloud,
	const string targetSensorName, const string cameraName, const cv::Mat &projImage, const bool draw, const bool ransac)
{
	//read calibration paraneters
	string a="/intrinsic_calibrations/ros_calib.yaml";
	string path = ros::package::getPath("calibration_gui");
	path += a;
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if(!fs.isOpened())
	{
		cout<<"failed to open document"<<endl;
		return -1;
	}

	// Matrices to store intrinsic parameters and distortion coefficients
	cv::Mat intrinsic_matrix, distortion_coeffs;

	fs["CM1"] >> intrinsic_matrix;
	fs["D1"] >> distortion_coeffs;

	// DEBUGGING =================================================================
	// cout << intrinsic_matrix << endl;
	// cout << distortion_coeffs << endl;
	//
	// cout << objectPoints << endl;
	// cout << imagePoints << endl;
	// DEBUGGING =================================================================

	string name = targetSensorName + "_" + cameraName + "_";

	cv::Mat rotation_vector(1,3,cv::DataType<double>::type);
	cv::Mat translation_vector(1,3,cv::DataType<double>::type);

	vector<cv::Point3f> objectPoints;
  vector<cv::Point2f> imagePoints;

	for (int i = 0; i < targetCloud.size(); i++)
	{
		objectPoints.push_back(cv::Point3f(targetCloud[i].x, targetCloud[i].y, targetCloud[i].z));
		imagePoints.push_back(cv::Point2f(cameraPnPCloud[i].x, cameraPnPCloud[i].y));
	}


	//rotation_vector = (cv::Mat_<double>(1,3) << 0, 0, 0); // DEBUGGING - solvePnP extrinsicGuess
	//translation_vector = (cv::Mat_<double>(1,3) << -1, 0, 2); // DEBUGGING - solvePnP extrinsicGuess

	/* SolvePnP brings points from the model coordinate system to the camera
	         coordinate system based on an array of points in the model coordinate space
	         (objectPoints) and an array of points in the camera coordinate image system
	         (imagePoints) */
	if (ransac)
	{
	solvePnPRansac(objectPoints, imagePoints,
		intrinsic_matrix,
		cv::noArray(),
		rotation_vector,
		translation_vector,
		false,
		100);
		name += "solvePnPRansac_";
	}
	else
	{
		solvePnP(objectPoints, imagePoints,
						intrinsic_matrix,
						cv::noArray(),
						//distortion_coeffs,
						rotation_vector,
						translation_vector,
						false,
						CV_ITERATIVE);
		name += "solvePnP_";
	}
	// No distortion coefficients are given because imagePoints are already undistorted

	// Project objectPoints to the image to check if solvePnP results are good ===
	vector<cv::Point2f> reprojectPoints;

	cv::projectPoints(objectPoints, rotation_vector, translation_vector,
	                  intrinsic_matrix,
	                  distortion_coeffs,
	                  reprojectPoints);

	//std::cout << reprojectPoints << std::endl; // DEBUGGING

	// SolvePnP re-projection error calculation
	float sum = 0.;
	sum = cv::norm(reprojectPoints, imagePoints);
	cout << "SolvePnP re-projection error = " << sum << endl;

	// Draw imagePoints and projectPoints on an image and save it
	cv::Mat projImageCLone = projImage.clone();
	int myradius=5;
	for (int i=0; i<reprojectPoints.size(); i++)
	{
    cv::line(projImageCLone, cv::Point(imagePoints[i].x - 5, imagePoints[i].y), cv::Point(imagePoints[i].x + 5, imagePoints[i].y), cv::Scalar(0,255,0), 2);  //crosshair horizontal
    cv::line(projImageCLone, cv::Point(imagePoints[i].x, imagePoints[i].y - 5), cv::Point(imagePoints[i].x, imagePoints[i].y + 5), cv::Scalar(0,255,0), 2);  //crosshair vertical
		//cv::circle(test_image, cv::Point(imagePoints[i].x, imagePoints[i].y), myradius, cv::Scalar(0,255,0),-1,8,0); // Green original points
	}
	imwrite( file_path + name + "imagePoints.jpg", projImageCLone );
	// Done with projected points ================================================
  for (int i=0; i<reprojectPoints.size(); i++)
	{
  cv::circle(projImageCLone, cv::Point(reprojectPoints[i].x, reprojectPoints[i].y), myradius, cv::Scalar(0,0,255),-1,8,0); // Red reprojected points
  }
  imwrite( file_path + name + "projectedPoints.jpg", projImageCLone );

	cv::Mat R(3,3,cv::DataType<double>::type);
	cv::Rodrigues(rotation_vector, R); // transforms rotation_vector in rotation matrix R (3x3)

	cv::Mat C_M(4,4,R.type()); // transformation matrix (4x4)

	C_M(cv::Range(0,3), cv::Range(0,3)) = R * 1; // copies R into T
	C_M(cv::Range(0,3), cv::Range(3,4)) = translation_vector * 1; // copies inv_translation_vector into T
	double *p = C_M.ptr<double>(3);
	p[0]=p[1]=p[2]=0; p[3]=1;

	cout << "SolvePnP Camera to LMS = " << C_M << endl; // writes camera to LMS transformation to terminal

	/* Invert transformation to get transformation from camera to model */
	R=R.t(); //For a rotation matrix it's transpose is equal to it's inverse
	cv::Mat inv_translation_vector;
	inv_translation_vector = -R * translation_vector; // translation_vector inverse


	cv::Mat T(4,4,R.type()); //Transformation matrix (4x4)
	T(cv::Range(0,3), cv::Range(0,3)) = R * 1; // copies R into T
	T(cv::Range(0,3), cv::Range(3,4)) = inv_translation_vector * 1; // copies inv_translation_vector into T

	p = T.ptr<double>(3);
	p[0]=p[1]=p[2]=0; p[3]=1;

	cout << "SolvePnP LMS to Camera = " << T << endl; // writes LMS to camera transformation to terminal

	// Save transformations on files
	string FilePath = file_path + name + "calib.txt";

	Matrix4f Trans;
	cv::cv2eigen(T, Trans);

	// writeFileCamera(C_M, "C_M", FilePath);
	// writeFileCamera(T, "M_C", FilePath);

	writeFile(Trans, FilePath);

	/* Visualization of camera position and orientation
	         Convert the opensv matrices to tf compatible ones */
	// Rotation matrices around the X axis so the pose represents the Z axis
	if (draw)
	{
		tf::Matrix3x3 rot;
		rot[0][0] = T.at<double>(0,0);
		rot[0][1] = T.at<double>(0,1);
		rot[0][2] = T.at<double>(0,2);
		rot[1][0] = T.at<double>(1,0);
		rot[1][1] = T.at<double>(1,1);
		rot[1][2] = T.at<double>(1,2);
		rot[2][0] = T.at<double>(2,0);
		rot[2][1] = T.at<double>(2,1);
		rot[2][2] = T.at<double>(2,2);
		tf::Vector3 trans(T.at<double>(0,3),
		                  T.at<double>(1,3),
		                  T.at<double>(2,3));

		tf::Quaternion q;
		rot.getRotation(q);
		camera.position.x=trans[0];
		camera.position.y=trans[1];
		camera.position.z=trans[2];
		camera.orientation.x=q[0];
		camera.orientation.y=q[1];
		camera.orientation.z=q[2];
		camera.orientation.w=q[3];
	}
	return 0;
}

/**
   @brief Creates a marker of the 3D car model to be displayed in Rviz
   @param[in] RPY vector containing (Roll, Pitch, Yaw) angles to rotate de model
   @param[in] translation vector containing (x, y, z) translations to translate the model
   @return marker is the 3D car model to displayed
 */
visualization_msgs::Marker addCar(const vector<double>& RPY, const vector<double>& translation)
{
  tf::Quaternion q;
	tf::Vector3 trans;

	if (translation.empty() && RPY.empty()) //user does not want to translate/rotate clouds and sensors.
	{
		trans = tf::Vector3( tfScalar(0), tfScalar(0), tfScalar(0) ); // no translation is done
		q = tf::createQuaternionFromRPY(0.0, 0.0, 0.0 ); // no rotation
	}
	else if (translation.empty()) // only rotation given by the user, no translation
	{
		trans = tf::Vector3( tfScalar(0), tfScalar(0), tfScalar(0) ); // no translation
		q = tf::createQuaternionFromRPY( RPY[0], RPY[1], RPY[2] ); // quaternion computation from given angles
	}
	else // rotation and translation given by the user
	{
		trans = tf::Vector3( tfScalar(translation[0]), tfScalar(translation[1]), tfScalar(translation[2]) ); // translation given by the user
		q = tf::createQuaternionFromRPY( RPY[0], RPY[1], RPY[2] ); // quaternion computation from given angles
	}

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/my_frame3";
	marker.header.stamp = ros::Time();
	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "ATLASCAR1";
	marker.id = 0;
	// Set the marker type
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = trans[0];
	marker.pose.position.y = trans[1];
	marker.pose.position.z = trans[2];
	marker.pose.orientation.x = q[0];
	marker.pose.orientation.y = q[1];
	marker.pose.orientation.z = q[2];//0.8836;
	marker.pose.orientation.w = q[3];//0.466;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;
	// Set the color
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.50;
	marker.color.g = 0.50;
	marker.color.b = 0.50;
	// Publish the marker
	marker.mesh_resource = "package://calibration_gui/../STL/ford_escort_atlascar1.stl";
	//car_pub.publish( marker );
  return marker;
}

/**
   @brief Computes the Euclidean distance between two points
   @param[in] p1 Point 1
   @param[in] p2 Point 2
   @return eu_dist Euclidean distance
 */
float pointEuclideanDistance (const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
{
	float eu_dist, x_dist, y_dist, z_dist;
	x_dist = p1.x - p2.x;
	y_dist = p1.y - p2.y;
	z_dist = p1.z - p2.z;
	eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );

	return eu_dist;
}

/**
   @brief Computes the Euclidean distance between two consecutive points in a point cloud
   @param[in] p1 Point cloud
   @return eu_dist_vector vector containing the Euclidean distance between each of the point pairs
 */
vector<float> gridEuclideanDistance ( const pcl::PointCloud<pcl::PointXYZ>& p1)
{
  float eu_dist, x_dist, y_dist, z_dist;
  vector<float> eu_dist_vector;
  // Euclidean distance calculation between points p1[i] and p1[i+1]
  for (int i=0; i<p1.points.size()-1; i++)
  {
		eu_dist = pointEuclideanDistance(p1[i], p1[i+1]);
    eu_dist_vector.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances
  }
  return eu_dist_vector;
}

/**
   @brief Computes the Euclidean distance between two points with the same index in different point clouds
   @param[in] p1 Point cloud 1
   @param[in] p2 Point cloud 2
   @return eu_dist_vector vector containing the Euclidean distance between each of the point pairs
 */
vector<float> pointCloudEuclideanDistance ( const pcl::PointCloud<pcl::PointXYZ>& p1, const pcl::PointCloud<pcl::PointXYZ>& p2)
{
  float eu_dist, x_dist, y_dist, z_dist;
  vector<float> eu_dist_vector;
  // Euclidean distance calculation between correspondent points of p1 and p2
  for (int i=0; i<p1.points.size(); i++)
  {
		eu_dist = pointEuclideanDistance(p1[i], p2[i]);
    eu_dist_vector.push_back(eu_dist); // Adds current euclidean distance to vector of euclidean distances
  }
  return eu_dist_vector;
}

/**
   @brief Computes the mean value of a vector
   @param[in] v vector of values
   @return mean of the vector
 */
float vectorMean ( const vector<float>& v )
{
  float sum = accumulate(v.begin(), v.end(), 0.0); // sums every element in "v"
  float mean = sum / v.size();  // mean value
  return mean;
}

/**
   @brief Write a file with the resulting calibration for cameras
   @param[in] v vector of values
   @return std populational standard deviation of vector \p v
 */
float vectorStdDeviationPopulation ( const vector<float>& v )
{
  float m = vectorMean (v); // mean calculation

  float sum_deviation = 0.0;
  for(int i=0; i<v.size(); ++i)
    sum_deviation+=(v[i]-m)*(v[i]-m); // adds errors between elements of the "v" and mean value

  float std = sqrt(sum_deviation / (v.size() )); // population standard deviation
  
  return std;
}
