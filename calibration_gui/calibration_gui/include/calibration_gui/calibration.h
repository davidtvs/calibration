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
#include <opencv2/core/eigen.hpp>

// Boost filesystem to get parent directory
#include "boost/filesystem.hpp"
#include <boost/bind.hpp>

using namespace Eigen;
using namespace std;
//using namespace cv; // causes errors because multiply defined FLANN headers on PCL and OpenCV

#if defined _CALIBRATION_UTILS_CPP_
string file_path;

#elif defined (_NODE_CPP_)
extern string file_path;
/**
  \class CircleCentroids
  \brief Class to handle the ball center coordinates from the different sensors
  \author David Silva
 */
class CircleCentroids
{
public:
/**
@brief Constructor. Subscription of the topics with the ball center coordinates in the different sensors
*/
    CircleCentroids(const vector<string> &sensors_list, const vector<bool> &isCamera)
    {
        //Topics I want to subscribe
        //Source: http://ros-users.122217.n3.nabble.com/How-to-identify-the-subscriber-or-the-topic-name-related-to-a-callback-td2391327.html
        string topic_name;
        int cam_count = 0;
        for (int i = 0; i < sensors_list.size(); i++)
        {
          topic_name = "/" + sensors_list[i] + "/BD_" + sensors_list[i] + "/SphereCentroid";
          cout << i << " " << topic_name << endl;
          pcl::PointXYZ allocator_ball_centers;
          allocator_ball_centers.x = -999;
          allocator_ball_centers.y = -999;
          allocator_ball_centers.z = -999;
          sensors_ball_centers.push_back(allocator_ball_centers);

          subs.push_back( n_.subscribe <geometry_msgs::PointStamped> (topic_name, 1, boost::bind(&CircleCentroids::sensorUpdate, this, _1, i)) );
        if (isCamera[i])
          {
            // Allocating space in camImage vector
            cv::Mat allocator_camImage;
            camImage.push_back(allocator_camImage);
            // Subscribing to raw image topics
            image_transport::ImageTransport it(n_);
            subs_cam_images.push_back( it.subscribe ("/" + sensors_list[i] + "/RawImage", 1, boost::bind(&CircleCentroids::imageUpdate, this, _1, cam_count)) );

            // Allocating space in camCentroidPnP vector
            pcl::PointXYZ allocator_camCentroidPnP;
            allocator_camCentroidPnP.x = -999;
            allocator_camCentroidPnP.y = -999;
            allocator_camCentroidPnP.z = -999;
            camCentroidPnP.push_back(allocator_camCentroidPnP);
            //Subscribing to topics containing image points for the solvepnp method
            subs_pnp.push_back( n_.subscribe <geometry_msgs::PointStamped> (topic_name + "PnP", 1, boost::bind(&CircleCentroids::camCentroidPnPUpdate, this, _1, cam_count)) );
            cout << "/" << sensors_list[i] << "/RawImage" << endl;
            cout << topic_name << "PnP" << endl;
            cam_count++;
          }
        }
    }

    ~CircleCentroids(){}

    // Source: https://foundry.supelec.fr/scm/viewvc.php/nouveau/ROS/koala_node/src/camera_position_node.cpp?view=markup&root=rpm_ims&sortdir=down&pathrev=2320
    void sensorUpdate(const geometry_msgs::PointStampedConstPtr& msg, const int i)
    {
      sensors_ball_centers[i].x = msg->point.x;
      sensors_ball_centers[i].y = msg->point.y;
      sensors_ball_centers[i].z = msg->point.z;
      // cout << "sensor callback: " << i << endl; //DEBUG
    }


    void camCentroidPnPUpdate(const geometry_msgs::PointStampedConstPtr& msg, int camNum)
    {
        camCentroidPnP[camNum].x = msg->point.x;
        camCentroidPnP[camNum].y = msg->point.y;
        camCentroidPnP[camNum].z = msg->point.z;
        // cout << "camera callback: " << camNum << endl; //DEBUG
    }


    void imageUpdate(const sensor_msgs::ImageConstPtr& msg, int camNum)
    {
      try
  		{
  			camImage[camNum] = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        // cout << "image callback: " << camNum << endl; //DEBUG
  		}
  		catch (cv_bridge::Exception &e)
  		{
  			ROS_ERROR("cv_bridge exception: %s", e.what());
  		}
    }

    vector<pcl::PointXYZ> getSensorsBallCenters (){ return sensors_ball_centers; }

    vector<pcl::PointXYZ> getCamCentroidPnP (){ return camCentroidPnP; }

    vector<cv::Mat> getCamImage (){ return camImage; }

  private:
    ros::NodeHandle n_;

    vector<ros::Subscriber> subs;
    vector<pcl::PointXYZ> sensors_ball_centers;

    vector<ros::Subscriber> subs_pnp;
    vector<pcl::PointXYZ> camCentroidPnP; /**< ball center coordinates on single camera image. */

    vector<image_transport::Subscriber> subs_cam_images;
    vector<cv::Mat> camImage;
};

#else
extern string file_path;
#endif

void createDirectory ( );
void writeFile(const Matrix4f transformation, const string filepath);
void writeFileCamera( cv::Mat transformation, const char* transformation_name, const string filepath);
void estimateTransformation(geometry_msgs::Pose & laser,pcl::PointCloud<pcl::PointXYZ> target_laserCloud,
  pcl::PointCloud<pcl::PointXYZ> & laserCloud, const string targetSensorName, const string sensorName);
int estimateTransformationCamera(geometry_msgs::Pose & camera, pcl::PointCloud<pcl::PointXYZ> targetCloud,
  pcl::PointCloud<pcl::PointXYZ> cameraPnPCloud , const string targetSensorName, const string cameraName, const cv::Mat &projImageconst, bool draw, const bool ransac);
visualization_msgs::Marker addCar(const vector<double>& RPY = vector<double>(), const vector<double>& translation = vector<double>() );
float pointEuclideanDistance (const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);
vector<float> gridEuclideanDistance ( const pcl::PointCloud<pcl::PointXYZ>& p1);
vector<float> pointCloudEuclideanDistance ( const pcl::PointCloud<pcl::PointXYZ>& p1, const pcl::PointCloud<pcl::PointXYZ>& p2);
float vectorMean ( const vector<float>& v );
float vectorStdDeviationPopulation ( const vector<float>& v );
#endif
