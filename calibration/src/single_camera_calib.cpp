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
   \file  camera_calib.cpp
   \brief Calibration of the stereo system using OpenCV
   \author Marcelo Pereira
   \date   December, 2015
 */

#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "FlyCapture2.h"
#include <cv.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include "calibration/camera_config.h"
#include <stdlib.h>

/**
   @brief Main function of the camera_calib node
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
	//PointGrey
	Error error;

	BusManager busManager;
	unsigned int numCameras;
	error = busManager.GetNumOfCameras(&numCameras);
	if(error != PGRERROR_OK)
	{
		cout<<"Failed to get number of cameras"<<endl;
		return false;
	}
	cout<<"number of cameras "<<numCameras<<endl;

	//Camera camera1, camera2;
	Camera Camera;

	PGRGuid pGuid;
	error = busManager.GetCameraFromIndex(0,&pGuid);
	if(error != PGRERROR_OK)
	{
		cout<<"Failed to get index of camera 1"<<endl;
		return false;
	}
	//cout<<"index camera 1 - "<<pGuid1.value<<endl;


	CameraInfo camInfo;

	//connect the two cameras
	error=Camera.Connect(&pGuid);
	if ( error != PGRERROR_OK )
	{
		cout << "Failed to connect to camera" <<endl;
		return false;
	}

	// Get the camera info and print it out
	error = Camera.GetCameraInfo( &camInfo );
	if ( error != PGRERROR_OK )
	{
		cout << "Failed to get camera info from camera" << endl;
		return false;
	}
	cout << camInfo.vendorName << " "
	     << camInfo.modelName << " "
	     << camInfo.serialNumber << endl;


	SetConfiguration(Camera);

	error = Camera.StartCapture();
	if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
	{
		cout << "Bandwidth exceeded" << endl;
		return false;
	}
	else if ( error != PGRERROR_OK )
	{
		cout << "Failed to start image capture" << endl;
		return false;
	}

	Mat imgGRAY1, imgGRAY2;

	// create the main window, and attach the trackbars
	namedWindow( "Camera 1", CV_WINDOW_NORMAL );

	int numBoards = 0;
	int numCornersHor;
	int numCornersVer;

	printf("Enter number of corners along width: ");
	scanf("%d", &numCornersHor);

	printf("Enter number of corners along height: ");
	scanf("%d", &numCornersVer);

	printf("Enter number of boards: ");
	scanf("%d", &numBoards);

	cout<<"Press Enter to Continue";
	cin.ignore();

	int numSquares = numCornersHor*numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);

	vector<vector<Point3f> > object_points;
	vector<vector<Point2f> > image_points;

	vector<Point2f> corners;
	int successes=0;

	vector<Point3f> obj;
	for(int j=0; j<numSquares; j++)
		obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

	Mat image1;

	// capture loop
	while(successes<numBoards)
	{
		// Get the image camera 1
		Image rawImage1;
		Error error = Camera.RetrieveBuffer( &rawImage1 );
		if ( error != PGRERROR_OK )
		{
			//cout << "capture error" << endl;
			continue;
		}

		// convert to rgb
		Image rgbImage1;
		rawImage1.Convert(PIXEL_FORMAT_BGR, &rgbImage1 );

		// convert to OpenCV Mat
		unsigned int rowBytes1 = (double)rgbImage1.GetReceivedDataSize()/(double)rgbImage1.GetRows();
		image1 = Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(),rowBytes1);

		cvtColor(image1, imgGRAY1, CV_BGR2GRAY);//convert the image from BGR to Gray scale

		bool found1 = findChessboardCorners(imgGRAY1, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		if(found1)
		{
			cornerSubPix(imgGRAY1, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(image1, board_sz, corners, found1);
			//cout<<"fouded chessboard"<<endl;

			image_points.push_back(corners);
			object_points.push_back(obj);

			cout<<"Snap stored!\n"<<endl;

			successes++;
			if(successes>=numBoards)
				break;
		}

		imshow("Camera 1", image1);

		int key = waitKey(1);
		if(key==27)
			return 0;

		sleep(5);
	}

	Mat intrinsic = Mat(3, 3, CV_64FC1);
	Mat distCoeffs;
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	Mat R, T, E, F;

	double rms = calibrateCamera(object_points,image_points,image1.size(),intrinsic,distCoeffs,rvecs,tvecs);

	cout<<"Matrix camera 1"<<endl;
	cout<<intrinsic<<endl;

	cout<<"distorcion camera 1"<<endl;
	cout<<distCoeffs<<endl;

  cout<<"Average Re-projection Error"<<endl;
  cout<<rms<<endl;



	FileStorage fs1("mystereocalib.yml", FileStorage::WRITE);
	fs1 << "CM1" << intrinsic;
	fs1 << "D1" << distCoeffs;

	fs1.release();

	//destroyWindow("Hough Circle Transform Demo"); //destroy the window with the name, "MyWindow"

	return 0;
}
