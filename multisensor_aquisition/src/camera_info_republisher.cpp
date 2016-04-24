/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
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

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
//#include <driver_base/driver.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class CameraInfoRepublisher
{
    public:
        
        CameraInfoRepublisher(ros::NodeHandle& nh_):
        nh(nh_),
        it(nh_)
        {
            ros::NodeHandle private_node("~");
            
            string camera_calibration_file;
            private_node.param("camera_calibration_file",camera_calibration_file,string("image calibration file not set"));
            
            string camera_name;
            private_node.param("camera_name",camera_name,string(""));
            
            info_manager = new camera_info_manager::CameraInfoManager(ros::NodeHandle(nh, camera_name),camera_name,camera_calibration_file);
            
            subscriber = it.subscribe("image_raw", 1, &CameraInfoRepublisher::imageHandler,this);
            
            publisher = it.advertiseCamera("new_info/image_raw", 1);
        }
  
        void imageHandler(const sensor_msgs::ImageConstPtr& msg)
        {
            image = *msg;
            info = info_manager->getCameraInfo();
            publisher.publish(image, info, image.header.stamp);
        }

        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        
        sensor_msgs::Image image;
        
        sensor_msgs::CameraInfo info;
        
        image_transport::Subscriber subscriber;
        
        camera_info_manager::CameraInfoManager* info_manager;
        
        image_transport::CameraPublisher publisher; 
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_info_republisher");
    ros::NodeHandle node;

    CameraInfoRepublisher info_publisher(node);
    
    ros::spin();
 
    return 0;
}
