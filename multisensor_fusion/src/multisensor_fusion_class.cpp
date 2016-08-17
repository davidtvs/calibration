/**************************************************************************************************
   Software License Agreement (BSD License)

   Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
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
   \file  multisensor_fusion_class.cpp
   \brief MultisensorFusion class methods
   \author David Silva
   \date   June, 2016
 */

#include "multisensor_fusion/multisensor_fusion_class.h"

/**
   @brief MultisensorFusion constructor. Reads geometric transformations to an eigen matrix and converts it to cvMat.
   @param[in] files is a vector of strings where each element is the filepath to a geometric transformation
 */
MultisensorFusion::MultisensorFusion(const std::vector<std::string> files)
{
	transformations.resize(files.size());
	tf_transforms.resize(files.size());
	tf_broadcasters.resize(files.size());

	for (int i=0; i < files.size(); i++)
	{
		readFile(transformations[i], files[i]);
	}

	eigenVector2cvVector(transformations);
}

/**
   @brief Read a file with geometric transformations
   @param[out] geometric transformation matrix
   @param[in] filepath and filename with extension
   @return void
 */
void MultisensorFusion::readFile( Eigen::Matrix4f &transformation, const std::string filepath)
{
	const char *file_path_dir = filepath.c_str(); // converts to char*

	int nrows = 4, ncols = 4;

	std::ifstream myfile;
	myfile.open (file_path_dir);
	if (myfile.is_open())
	{
		for (int row = 0; row < nrows; row++)
			for (int col = 0; col < ncols; col++)
			{
				float num = 0.0;
				myfile >> num;
				transformation(row, col) = num;
			}
		myfile.close();
	}

	std::cout << "Read file: " << filepath << std::endl;
	std::cout << transformation << std::endl;

}

/**
   @brief Converts a vector of Eigen matrices to a vector of cvMat.
   @param[in] eigenVector vector of geometric transformations in Eigen matrices
   @return void
 */
void MultisensorFusion::eigenVector2cvVector(std::vector<Eigen::Matrix4f> eigenVector)
{
	cv::Mat cvTransf_tmp(4,4,CV_32FC1);
	for (int i=0; i < eigenVector.size(); i++)
	{
		cv::eigen2cv(eigenVector[i], cvTransf_tmp);
		cvTransf.push_back(cvTransf_tmp.clone());
		std::cout << cvTransf[i] << std::endl;
	}
}

/**
   @brief Gets transformations in TF format from transformations in Eigen format
   @param void
   @return tf_transforms is a vector of transformations in TF format
 */
std::vector<tf::Transform> MultisensorFusion::getTFTranforms()
{
	tf::Matrix3x3 T;
	tf::Quaternion q;
	for (int i=0; i < tf_transforms.size(); i++)
	{
		tf_transforms[i].setOrigin(tf::Vector3(transformations[i](0,3),
		                                       transformations[i](1,3),
		                                       transformations[i](2,3)));

		T.setValue(transformations[i](0,0),transformations[i](0,1),transformations[i](0,2),
		           transformations[i](1,0),transformations[i](1,1),transformations[i](1,2),
		           transformations[i](2,0),transformations[i](2,1),transformations[i](2,2));


		T.getRotation(q);

		tf_transforms[i].setRotation(q);
	}

	return tf_transforms;
}

/**
   @brief Gets the vector of TF broadcasters.
   @param void
   @return tf_broadcasters is the vector of TF broadcasters
 */
std::vector<tf::TransformBroadcaster> MultisensorFusion::getTFBroadcasters()
{
	return tf_broadcasters;
}

/**
   @brief Gets transformations in Eigen format.
   @param void
   @return transformations vector of Eigen transformations
 */
std::vector<Eigen::Matrix4f> MultisensorFusion::getEigenTransformations ()
{
	return transformations;
}

/**
   @brief Gets transformations in cvMat format.
   @param void
   @return cvTransf vector of cvMat transformations
 */
std::vector<cv::Mat> MultisensorFusion::getCVTransformations ()
{
	return cvTransf;
}

/**
   @brief Projects 3D world points (from lasers, Kinect, Swissranger, etc) to an image.
   @param[in] img initial image
   @param[in] cvTransformation geometric transformation between the sensor and the camera
   @param[in] points_3D 3D points acquired by the sensor
   @param[in] intrinsic_matrix intrinc camera parameters
   @param[in] distortion_coeffs tangential and radial lens distortion coefficients
   @param[in] color is the color used to draw the 3D points on the image
   @return imgClone is the initial image with the projected 3D points
 */
cv::Mat MultisensorFusion::project3DtoImage(const cv::Mat img, cv::Mat cvTransformation,
	std::vector<cv::Point3f> points_3D, cv::Mat intrinsic_matrix, cv::Mat distortion_coeffs, cv::Scalar color)
{
	cv::Mat rotation_vector(1,3,cv::DataType<double>::type);
	cv::Mat translation_vector(1,3,cv::DataType<double>::type);

	// Computes Rodrigues rotation vector for cvTransformation
	cv::Mat R(3,3,cv::DataType<double>::type);
	R = cvTransformation(cv::Range(0,3), cv::Range(0,3));
	cv::Rodrigues(R, rotation_vector);

	//std::cout << "Rodrigues rotation vector = " << rotation_vector << std::endl;

	// Computes translation vector for cvTransformation
	translation_vector = cvTransformation(cv::Range(0,3), cv::Range(3,4));

	//std::cout << "Translation vector = " << translation_vector << std::endl;

	// Project objectPoints to the image to check if solvePnP results are good ===
	std::vector<cv::Point2f> reprojectPoints;

	cv::projectPoints(points_3D, rotation_vector, translation_vector,
	                  intrinsic_matrix,
	                  distortion_coeffs,
	                  reprojectPoints);

	cv::Mat imgClone = img.clone();
	int myradius=2;
	for (int i=0; i<reprojectPoints.size(); i++)
	{
		if (reprojectPoints[i].x >= 0 && reprojectPoints[i].x <= imgClone.cols && reprojectPoints[i].y >= 0 && reprojectPoints[i].y <= imgClone.rows)
			cv::circle(imgClone, cv::Point(reprojectPoints[i].x, reprojectPoints[i].y), myradius, color,-1,8,0); // Red reprojected points
	}

	return imgClone;
}

/**
   @brief Creates a marker of the 3D car model to be displayed in Rviz
   @param[in] RPY vector containing (Roll, Pitch, Yaw) angles to rotate de model
   @param[in] translation vector containing (x, y, z) translations to translate the model
   @param[in] frameID Rviz frame name
   @return marker is the 3D car model to displayed
 */
visualization_msgs::Marker MultisensorFusion::addCar(const std::vector<double> RPY, const std::vector<double> translation, const std::string frameID)
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
	marker.header.frame_id = frameID;
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
	marker.mesh_resource = "package://multisensor_fusion/STL/ford_escort_atlascar1.stl";
	//car_pub.publish( marker );
  return marker;
}