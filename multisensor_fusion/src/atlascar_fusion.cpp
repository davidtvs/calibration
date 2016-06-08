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

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <fstream>

#include <eigen3/Eigen/Dense>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "atlascar_fusion");
	ros::NodeHandle node;

	std::string pkg_path = ros::package::getPath("multisensor_fusion"); // get calibration package path

	std::string file_path = pkg_path + "/geometric_transf/atlascar/lms151_1_ldmrs_1_calib.txt"; // complete filepath where results will be saved

	const char *file_path_dir = file_path.c_str(); // converts to char*

	Eigen::Matrix4f transformation;
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

	std::cout << transformation << std::endl;

	tf::Matrix3x3 T;
	T.setValue(transformation(0,0),transformation(0,1),transformation(0,2),
						 transformation(1,0),transformation(1,1),transformation(1,2),
						 transformation(2,0),transformation(2,1),transformation(2,2));



	tf::Quaternion q;

	// LMS151 reference =========================================================
	tf::TransformBroadcaster br_lms151a;
	tf::Transform transform_lms151a;

	transform_lms151a.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	q.setRPY(0, 0, 0);

	transform_lms151a.setRotation(q);

	// LMS151 non-reference =====================================================
	tf::TransformBroadcaster br_lms151b;
	tf::Transform transform_lms151b;

	transform_lms151b.setOrigin(tf::Vector3(transformation(0,3),
																					transformation(1,3),
																					transformation(2,3)));

	T.getRotation(q);

	transform_lms151b.setRotation(q);


	ros::Rate rate(10.0);
	while (node.ok())
	{
		br_lms151a.sendTransform(tf::StampedTransform(transform_lms151a, ros::Time::now(),"atlascar_fusion","lms151_1"));

		br_lms151b.sendTransform(tf::StampedTransform(transform_lms151b, ros::Time::now(),"atlascar_fusion","lms151_2"));

		rate.sleep();
	}

	return 0;
}
