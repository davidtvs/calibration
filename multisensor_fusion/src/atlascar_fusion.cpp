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
   \file  atlascar_fusion.cpp
   \brief
   \details
   \author David Silva
   \date   June, 2016
 */

#include "multisensor_fusion/atlascar_fusion.h"
#include "multisensor_fusion/multisensor_fusion_class.h"
#include <colormap/colormap.h>

//OpenCV
//#include <opencv2/highgui/highgui.hpp>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "atlascar_fusion");
	ros::NodeHandle node;

	// get calibration package path and navigate to folder where the geometric transformations are kept
	std::string pkg_path = ros::package::getPath("multisensor_fusion") + "/geometric_transf/atlascar/";

	std::vector<std::string> files;

	// Get transformation for each sensor
	std::string file_path;
	// 3D rigid transforms
	file_path = pkg_path + "lms151_1_lms151_2_calib.txt";
	files.push_back(file_path);

	file_path = pkg_path + "lms151_1_ldmrs_1_calib.txt";
	files.push_back(file_path);

	file_path = pkg_path + "lms151_1_pointgrey_1_calib.txt";
	files.push_back(file_path);

	// Extrinsic camera calibration
	file_path = pkg_path + "lms151_1_pointgrey_1_solvePnPRansac_calib.txt";
	files.push_back(file_path);


	//read calibration paraneters
	std::string path = ros::package::getPath("multisensor_fusion") + "/intrinsic_calibrations/ros_calib.yaml";
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if(!fs.isOpened())
	{
		std::cout << "failed to intrisic parameters file" << std::endl;
		return -1;
	}

	// Matrices to store intrinsic parameters and distortion coefficients
	cv::Mat intrinsic_matrix, distortion_coeffs;

	fs["CM"] >> intrinsic_matrix;
	fs["D"] >> distortion_coeffs;


	MultisensorFusion fusion_helper(files);

	MultisensorSubs multisensor_subs;


	// LMS151 - reference sensor ================================================
	tf::Quaternion q;
	tf::TransformBroadcaster br_lms151a;
	tf::Transform transform_lms151a;

	transform_lms151a.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	q.setRPY(0, 0, 0);

	transform_lms151a.setRotation(q);

	// Non-reference sensors ====================================================
	std::vector<tf::Transform> tf_transforms = fusion_helper.getTFTranforms();
	std::vector<tf::TransformBroadcaster> tf_broadcasters = fusion_helper.getTFBroadcasters();


	std::vector<cv::Mat> transformations = fusion_helper.getCVTransformations();

	cv::Mat pointgrey_lms151_1 = transformations[2].inv();
	cv::Mat pointgrey_lms151_2 = transformations[2].inv() * transformations[0];
	cv::Mat pointgrey_ldmrs = transformations[2].inv() * transformations[1];

	// Create a colormap
	class_colormap colormap("hsv", 3, 1, false);

	std::vector<cv::Point2f> reprojectPoints;
	cv::Mat fusion_img = multisensor_subs.camImage.clone();

	ros::Rate rate(10.0);
	while (node.ok())
	{
		br_lms151a.sendTransform(tf::StampedTransform(transform_lms151a, ros::Time::now(),"atlascar_fusion","lms151_1"));

		tf_broadcasters[0].sendTransform(tf::StampedTransform(tf_transforms[0], ros::Time::now(),"atlascar_fusion","lms151_2"));

		tf_broadcasters[1].sendTransform(tf::StampedTransform(tf_transforms[1], ros::Time::now(),"atlascar_fusion","ldmrs"));
		

		fusion_img = fusion_helper.project3DtoImage(fusion_img, pointgrey_lms151_1, multisensor_subs.lms1CVpoints,
			intrinsic_matrix, distortion_coeffs, colormap.cv_color(0));

		fusion_img = fusion_helper.project3DtoImage(fusion_img, pointgrey_lms151_2, multisensor_subs.lms2CVpoints,
			intrinsic_matrix, distortion_coeffs, colormap.cv_color(1));

		fusion_img = fusion_helper.project3DtoImage(fusion_img, pointgrey_ldmrs, multisensor_subs.ldmrsCVpoints,
			intrinsic_matrix, distortion_coeffs, colormap.cv_color(2));

		rate.sleep();
	}

	return 0;
}
