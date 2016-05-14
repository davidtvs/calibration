/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
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
\file  calibration_utils.h
\brief Global include file
\author David Silva
*/

#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <vector>
#include <numeric> // for mean and standard deviation calculation (accumulate)

// ROS
#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include "tf/tf.h"
#include <geometry_msgs/Pose.h>
// To subscribe to image
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>

//OpenCV
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>

// Boost filesystem to get parent directory
#include "boost/filesystem.hpp"

#define BALL_DIAMETER 0.99

using namespace Eigen;
using namespace std;
//using namespace cv; // causes errors because multiply defined FLANN headers on PCL and OpenCV

#if defined _CALIBRATION_UTILS_CPP_
string file_path;

#elif defined _CALIBRATION_CPP_
extern string file_path;
/**
  \class CircleCentroids
  \brief Class to handle the ball center coordinates from the different sensors
  \author Marcelo Pereira
 */
class CircleCentroids
{
public:
    ros::NodeHandle n_;
    ros::Subscriber lms1Centroid_subscriber;
    ros::Subscriber lms2Centroid_subscriber;
    ros::Subscriber ldmrsCentroid_subscriber;
    ros::Subscriber swissrangerCentroid_subscriber;
    ros::Subscriber cam1Centroid_subscriber;
    ros::Subscriber singleCamCentroid_subscriber;
    ros::Subscriber singleCamCentroidPnP_subscriber;
    image_transport::Subscriber image_subscriber;
    geometry_msgs::PointStamped lms1Centroid; /**< ball center coordinates on the first sick lms151 laser. */
    geometry_msgs::PointStamped lms2Centroid; /**< ball center coordinates on the second sick lms151 laser. */
    geometry_msgs::PointStamped ldmrsCentroid; /**< ball center coordinates on the sick ld-mrs laser. */
    geometry_msgs::PointStamped swissrangerCentroid; /**< ball center coordinates on the swissranger. */
    geometry_msgs::PointStamped cam1Centroid; /**< ball center coordinates on the stereo system. */
    geometry_msgs::PointStamped singleCamCentroid; /**< ball center coordinates on single camera coordinate system. */
    geometry_msgs::PointStamped singleCamCentroidPnP; /**< ball center coordinates on single camera image. */
    sensor_msgs::ImageConstPtr camImage;

/**
@brief constructer - subscription of the topics with the ball center coordinates in the different sensors
*/
    CircleCentroids()
    {
        //Topics I want to subscribe
        lms1Centroid_subscriber=n_.subscribe("/laser1/sphereCentroid", 1000, &CircleCentroids::lms1CentroidUpdate, this);
        lms2Centroid_subscriber=n_.subscribe("/laser2/sphereCentroid", 1000, &CircleCentroids::lms2CentroidUpdate, this);
        ldmrsCentroid_subscriber=n_.subscribe("/sphere_centroid", 1000, &CircleCentroids::ldmrsCentroidUpdate, this);
        swissrangerCentroid_subscriber=n_.subscribe("/swissranger/spherecenter", 1000, &CircleCentroids::swissrangerCentroidUpdate, this);
        cam1Centroid_subscriber=n_.subscribe("/Camera1/ballCentroid",1000, &CircleCentroids::cam1CentroidUpdate, this);
        singleCamCentroid_subscriber = n_.subscribe("/SingleCamera/ballCentroid", 1, &CircleCentroids::singleCamCentroidUpdate, this);
        singleCamCentroidPnP_subscriber = n_.subscribe("/SingleCamera/ballCentroidPnP", 1, &CircleCentroids::singleCamCentroidPnPUpdate, this);
        image_transport::ImageTransport it(n_);
        image_subscriber = it.subscribe("/SingleCamera/image", 1, &CircleCentroids::imageUpdate, this);
    }

    ~CircleCentroids(){}

    void lms1CentroidUpdate(const geometry_msgs::PointStamped& msg)
    {
        lms1Centroid=msg;
    }

    void lms2CentroidUpdate(const geometry_msgs::PointStamped& msg)
    {
        lms2Centroid=msg;
    }

    void ldmrsCentroidUpdate(const geometry_msgs::PointStamped& msg)
    {
        ldmrsCentroid=msg;
    }

    void swissrangerCentroidUpdate(const geometry_msgs::PointStamped& msg)
    {
        swissrangerCentroid=msg;
    }

    void cam1CentroidUpdate(const geometry_msgs::PointStamped& msg)
    {
        cam1Centroid=msg;
    }

    void singleCamCentroidUpdate(const geometry_msgs::PointStamped& msg)
    {
        singleCamCentroid=msg;
    }
    void singleCamCentroidPnPUpdate(const geometry_msgs::PointStamped& msg)
    {
        singleCamCentroidPnP=msg;
    }
    void imageUpdate(const sensor_msgs::ImageConstPtr& msg)
    {
      camImage = msg;
    }
};

#else
extern string file_path;
#endif

void createDirectory ( );
void writeFile(pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation, const string filepath);
void writeFileCamera( cv::Mat transformation, const char* transformation_name, const string filepath);
void estimateTransformation(geometry_msgs::Pose & laser,pcl::PointCloud<pcl::PointXYZ> target_laserCloud, pcl::PointCloud<pcl::PointXYZ> & laserCloud, const string laserNames);
int estimateTransformationCamera(geometry_msgs::Pose & camera, vector<cv::Point3f> objectPoints, vector<cv::Point2f> imagePoints, const string name, const bool draw = false, const bool ransac = false);
visualization_msgs::Marker addCar(const vector<double>& RPY = vector<double>(), const vector<double>& translation = vector<double>() );
vector<float> gridEuclideanDistance ( const pcl::PointCloud<pcl::PointXYZ>& p1);
vector<float> pointCloudEuclideanDistance ( const pcl::PointCloud<pcl::PointXYZ>& p1, const pcl::PointCloud<pcl::PointXYZ>& p2);
float vectorMean ( const vector<float>& v );
float vectorStdDeviationPopulation ( const vector<float>& v );
#endif
