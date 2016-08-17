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
   \file  pointgrey_fl3-ge-28s4-c_driver.cpp
   \brief Driver node for the Point Grey FL3-GE28S4-C. Sets a camera configuration, grabs the images and publishes them to a topic named RawImage.
   Camera configuration:
   - Resolution: 964x724 pixels
   - Offset: x=0, y=0
   - Mode: Mode 1
   \author David Silva
   \date   August, 2016
 */

#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "FlyCapture2.h"
#include <cv.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>

using namespace cv;
using namespace std;
using namespace FlyCapture2;

/**
   @brief Prints FlyCapture2 version and build date
   @param void
   @return void
 */
void PrintBuildInfo()
{
	FC2Version fc2Version;
	Utilities::GetLibraryVersion( &fc2Version );

	ostringstream version;
	version << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build;
	cout << version.str() << endl;

	ostringstream timeStamp;
	timeStamp <<"Application build date: " << __DATE__ << " " << __TIME__;
	cout << timeStamp.str() << endl << endl;
}

/**
   @brief Prints camera information:
   - Serial number;
   - Camera model;
   - Camera vendor;
   - Sensor;
   - Resolution;
   - Firmware version;
   - Firmware build time.
   @param[in] pCamInfo camera info
   @return void
 */
void PrintCameraInfo( CameraInfo* pCamInfo )
{
	cout << endl;
	cout << "*** CAMERA INFORMATION ***" << endl;
	cout << "Serial number -" << pCamInfo->serialNumber << endl;
	cout << "Camera model - " << pCamInfo->modelName << endl;
	cout << "Camera vendor - " << pCamInfo->vendorName << endl;
	cout << "Sensor - " << pCamInfo->sensorInfo << endl;
	cout << "Resolution - " << pCamInfo->sensorResolution << endl;
	cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
	cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;

}

/**
   @brief Print error trace
   @param[in] error
   @return void
 */
void PrintError( Error error )
{
	error.PrintErrorTrace();
}

/**
@brief Configuration of the camera image format
- Resolution: 964x724
- Offset: x=0, y=0
- Mode: Mode 1
@param[in] camera
@return false if an error occurred, true on success
*/
bool SetConfiguration(Camera &camera)
{

    uint16_t roi_width;
    uint16_t roi_height;
    uint16_t roi_offset_x;
    uint16_t roi_offset_y;

    // Error for checking if functions went okay
    Error error;

    // Get Format7 information
    Format7Info fmt7Info;
    bool supported;
    //fmt7Info.mode = fmt7Mode;
    error = camera.GetFormat7Info(&fmt7Info, &supported);
    if ( error != PGRERROR_OK )
    {
      PrintError( error );
      return false;
    }

    // Make Format7 Configuration
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = MODE_1;
    fmt7ImageSettings.pixelFormat = PIXEL_FORMAT_RAW8;

    // Check Width
    roi_width = 964; // Locks the width into an appropriate multiple using an integer divide

    fmt7ImageSettings.width = roi_width;


    // Check Height
    roi_height = 724; // Locks the height into an appropriate multiple using an integer divide

    fmt7ImageSettings.height = roi_height;


    // Check OffsetX
    roi_offset_x = 0;  // Locks the X offset into an appropriate multiple using an integer divide

    fmt7ImageSettings.offsetX  = roi_offset_x;

    // Check OffsetY
    roi_offset_y = 0;  // Locks the X offset into an appropriate multiple using an integer divide

    fmt7ImageSettings.offsetY  = roi_offset_y;

    // Validate the settings to make sure that they are valid
    Format7PacketInfo fmt7PacketInfo;
    bool valid;
    error = camera.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);
    if ( error != PGRERROR_OK )
    {
      PrintError( error );
      return false;
    }

    // Stop the camera to allow settings to change.
    error = camera.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);
    if ( error != PGRERROR_OK )
    {
      PrintError( error );
      return false;
    }

    // Get camera info to check if camera is running in color or mono mode
    CameraInfo cInfo;
    error = camera.GetCameraInfo(&cInfo);
    if ( error != PGRERROR_OK )
    {
      PrintError( error );
      return false;
    }
	return true;
}


/**
   @brief Driver node for the Point Grey FL3-GE28S4-C
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Point_Grey");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Publisher rawImage_pub = it.advertise("RawImage", 1);

	sensor_msgs::ImagePtr image_msg;

	//PointGrey
	Error error;

	BusManager busManager;
	unsigned int numCameras=0, count=0;
	while(numCameras!=1 && ros::ok())
	{
		error = busManager.GetNumOfCameras(&numCameras);
		if(error != PGRERROR_OK || count==10)
		{
			PrintError( error );
			return -1;
		}
		cout<<numCameras<<"cameras detected"<<endl;
		count++;
		ros::Duration(2).sleep(); // sleep for two seconds
	}
	cout<<"number of cameras "<<numCameras<<endl;

	//Camera camera1, camera2;
	Camera Camera;

	PGRGuid pGuid;
	error = busManager.GetCameraFromIndex(0,&pGuid);
	if(error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	//cout<<"index camera 1 - "<<pGuid1.value<<endl;

	//connect the two cameras
	error=Camera.Connect(&pGuid);
	if ( error != PGRERROR_OK )
	{
		PrintError( error );
		return -1;
	}

	CameraInfo camInfo;
	// Get the camera info and print it out
	error = Camera.GetCameraInfo( &camInfo );
	if ( error != PGRERROR_OK )
	{
		PrintError( error );
		return -1;
	}

	PrintCameraInfo(&camInfo);

	SetConfiguration(Camera);

	error = Camera.StartCapture();
	if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
	{
		PrintError( error );
		return -1;
	}
	else if ( error != PGRERROR_OK )
	{
		PrintError( error );
		return -1;
	}


	Mat image1;
	Image rawImage1;
	Image rgbImage1;
	// capture loop
	//ros::Rate loop_rate(15);
	// capture loop
	while(ros::ok())
	{

		ros::Time start = ros::Time::now();

		// Get the image camera 1
		error = Camera.RetrieveBuffer( &rawImage1 );
		if ( error != PGRERROR_OK )
		{
			PrintError( error );
			continue;
		}

		// convert to rgb
		error = rawImage1.Convert(PIXEL_FORMAT_BGR, &rgbImage1 );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		// convert to OpenCV Mat
		unsigned int rowBytes1 = (double)rgbImage1.GetReceivedDataSize()/(double)rgbImage1.GetRows();
		image1 = Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(),rowBytes1);

		if(!image1.empty()) {
			image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image1).toImageMsg();
			rawImage_pub.publish(image_msg);
		}
		//ros::spinOnce();
		//loop_rate.sleep();

		ros::Time end = ros::Time::now();

		cout << "Image getter: " <<  (end - start).toNSec() * 1e-6 << " msec"  << endl;
	}

	// Stop capturing images
	error = Camera.StopCapture();
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}

	// Disconnect the camera
	error = Camera.Disconnect();
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}

	return 0;
}
