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
   \file  multisensor_fusion_class.h
   \brief Header file for multisensor_fusion_class.cpp
   \author David Silva
   \date   June, 2016
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

#include <eigen3/Eigen/Dense>

//OpenCV
#include <opencv2/core/eigen.hpp>
#include "opencv2/calib3d/calib3d.hpp"

/**
   \class MultisensorFusion
   \brief Helper class to perform data fusion.
   \author David Silva
 */
class MultisensorFusion
{
public:
	/**
	   @brief constructer -
	 */
	MultisensorFusion(const std::vector<std::string> files);

	~MultisensorFusion(){}

	std::vector<tf::Transform> getTFTranforms();

	std::vector<tf::TransformBroadcaster> getTFBroadcasters();

	std::vector<Eigen::Matrix4f> getEigenTransformations();

  std::vector<cv::Mat> getCVTransformations ();

  cv::Mat project3DtoImage(const cv::Mat img, cv::Mat cvTransformation, std::vector<cv::Point3f> points_3D,
    cv::Mat intrinsic_matrix, cv::Mat distortion_coeffs, cv::Scalar color);

	visualization_msgs::Marker addCar(const std::vector<double> RPY, const std::vector<double> translation, const std::string frameID);

private:

	std::vector<Eigen::Matrix4f> transformations;
	std::vector<tf::Transform> tf_transforms;
	std::vector<tf::TransformBroadcaster> tf_broadcasters;
	std::vector<cv::Mat> cvTransf;


	void readFile( Eigen::Matrix4f &transformation, const std::string filepath);

	void eigenVector2cvVector(std::vector<Eigen::Matrix4f> eigenVector);
};
