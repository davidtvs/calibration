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
   \brief Performs data fusion between the sensors used on ATLASCAR 1:
   - 2 SICK LMS151;
   - SICK LD-MRS400001;
   - Point Grey FL3-GE28S4-C.
   \author David Silva
   \date   June, 2016
 */

#include "multisensor_fusion/subscribers_class.h"
#include "multisensor_fusion/multisensor_fusion_class.h"


/**
   @brief Main function which performs data fusion with the following sensors:
   - 2 SICK LMS151;
   - SICK LD-MRS400001;
   - Point Grey FL3-GE28S4-C.
   @param argc
   @param argv
   @return int
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "kinect_pg_lms1_lms2_mrs");
	ros::NodeHandle node;

	ros::Publisher car_pub;
	car_pub = node.advertise<visualization_msgs::Marker>("/ATLASCAR1", 1);

	image_transport::ImageTransport it(node);
	image_transport::Publisher fusion_image_pub = it.advertise("image_fusion", 3);
	sensor_msgs::ImagePtr image_msg;
	image_transport::Publisher fusion_image_pnp_pub = it.advertise("image_fusion_pnp", 3);
	sensor_msgs::ImagePtr image_msg_pnp;

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

	std::vector<double> RPY;
	// ATLASCAR model rotations
	RPY.push_back(M_PI/2); // X-rotation
	RPY.push_back(0.0); // Y-rotation
	RPY.push_back(M_PI); // Z-rotation

	std::vector<double> translation;
	translation.push_back(-4.387/2); // X translation. 4.387 is the car's length
	translation.push_back(-1.702/2); // Y translation. 1.702 is the car's width
	translation.push_back(-0.46); // Z translation. 0.46 is the height of the reference LMS sensor

	visualization_msgs::Marker atlascar = fusion_helper.addCar(RPY, translation, "/atlascar_fusion"); // builds the marker to publish



	MultisensorSubs multisensor_subs(intrinsic_matrix, distortion_coeffs);

	multisensor_subs.setLMSAngleLimits(-135, 45, -45, 135);



	tf::Transform t0;
  tf::Quaternion q0;
	t0.setOrigin( tf::Vector3( tfScalar(0), tfScalar(0), tfScalar(0)) ); // no translation is done
	q0 = tf::createQuaternionFromRPY(0, 0, 55 * M_PI/180); // no rotation
	t0.setRotation( q0 );


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

	cv::Mat pointgrey_lms151_1_pnp = transformations[3].inv();
	cv::Mat pointgrey_lms151_2_pnp = transformations[3].inv() * transformations[0];
	cv::Mat pointgrey_ldmrs_pnp = transformations[3].inv() * transformations[1];

	cv::Mat fusion_img;
	cv::Mat fusion_img_pnp;

	ros::Rate rate(15.0);
	while (node.ok())
	{
		br_lms151a.sendTransform(tf::StampedTransform(t0*transform_lms151a, ros::Time::now(),"atlascar_fusion","lms151_1"));

		tf_broadcasters[0].sendTransform(tf::StampedTransform(t0*tf_transforms[0], ros::Time::now(),"atlascar_fusion","lms151_2"));

		tf_broadcasters[1].sendTransform(tf::StampedTransform(t0*tf_transforms[1], ros::Time::now(),"atlascar_fusion","ldmrs"));

		fusion_img = multisensor_subs.camImage_rect.clone();
		fusion_img_pnp = multisensor_subs.camImage_rect.clone();

		if (multisensor_subs.lms1CVpoints.size())
		{
			fusion_img = fusion_helper.project3DtoImage(fusion_img, pointgrey_lms151_1, multisensor_subs.lms1CVpoints,
			                                            intrinsic_matrix, distortion_coeffs, cv::Scalar( 0, 0, 255));

			fusion_img_pnp = fusion_helper.project3DtoImage(fusion_img_pnp, pointgrey_lms151_1_pnp, multisensor_subs.lms1CVpoints,
			                                                intrinsic_matrix, distortion_coeffs, cv::Scalar( 0, 0, 255));
		}
		if (multisensor_subs.lms2CVpoints.size())
		{
			fusion_img = fusion_helper.project3DtoImage(fusion_img, pointgrey_lms151_2, multisensor_subs.lms2CVpoints,
			                                            intrinsic_matrix, distortion_coeffs, cv::Scalar( 0, 255, 0));

			fusion_img_pnp = fusion_helper.project3DtoImage(fusion_img_pnp, pointgrey_lms151_2_pnp, multisensor_subs.lms2CVpoints,
			                                            intrinsic_matrix, distortion_coeffs, cv::Scalar( 0, 255, 0));
		}
		if (multisensor_subs.ldmrsCVpoints.size())
		{
			fusion_img = fusion_helper.project3DtoImage(fusion_img, pointgrey_ldmrs, multisensor_subs.ldmrsCVpoints,
			                                            intrinsic_matrix, distortion_coeffs, cv::Scalar( 255, 255, 0));

			fusion_img_pnp = fusion_helper.project3DtoImage(fusion_img_pnp, pointgrey_ldmrs_pnp, multisensor_subs.ldmrsCVpoints,
			                                            intrinsic_matrix, distortion_coeffs, cv::Scalar( 255, 255, 0));
		}
		if(!fusion_img.empty()) {
			image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", fusion_img).toImageMsg();
			fusion_image_pub.publish(image_msg);
			image_msg_pnp = cv_bridge::CvImage(std_msgs::Header(), "bgr8", fusion_img_pnp).toImageMsg();
			fusion_image_pnp_pub.publish(image_msg_pnp);
		}

		car_pub.publish( atlascar );

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
