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
   \brief
   \details
   \author David Silva
   \date   June, 2016
 */

#include "multisensor_fusion/multisensor_fusion_class.h"

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
   @param[in] geometric transformation matrix
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

std::vector<tf::TransformBroadcaster> MultisensorFusion::getTFBroadcasters()
{
	return tf_broadcasters;
}

std::vector<Eigen::Matrix4f> MultisensorFusion::getEigenTransformations ()
{
	return transformations;
}

std::vector<cv::Mat> MultisensorFusion::getCVTransformations ()
{
	return cvTransf;
}

cv::Mat MultisensorFusion::project3DtoImage(const cv::Mat img, cv::Mat cvTransformation,
	std::vector<cv::Point3f> points_3D, cv::Mat intrinsic_matrix, cv::Mat distortion_coeffs, cv::Scalar color)
{
	cv::Mat rotation_vector(1,3,cv::DataType<double>::type);
	cv::Mat translation_vector(1,3,cv::DataType<double>::type);

	// Computes Rodrigues rotation vector for cvTransformation
	cv::Mat R(3,3,cv::DataType<double>::type);
	R = cvTransformation(cv::Range(0,3), cv::Range(0,3));
	cv::Rodrigues(R, rotation_vector);

	std::cout << "Rodrigues rotation vector = " << rotation_vector << std::endl;

	// Computes translation vector for cvTransformation
	translation_vector = cvTransformation(cv::Range(0,3), cv::Range(3,4));

	std::cout << "Translation vector = " << translation_vector << std::endl;

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


//
// // LMS151 reference =========================================================
// tf::TransformBroadcaster br_lms151a;
// tf::Transform transform_lms151a;
//
// transform_lms151a.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
// q.setRPY(0, 0, 0);
//
// transform_lms151a.setRotation(q);
//
// // LMS151 non-reference =====================================================
// tf::TransformBroadcaster br_lms151b;
