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
   \file  camera.cpp
   \brief Ball detection with a stereo system
   \author Marcelo Pereira
   \date   December, 2015
 */

#include "calibration/camera_config.h"


//Marker's publisher
ros::Publisher ballCentroidCam_pub;
ros::Publisher ballCentroidCamPnP_pub;
ros::Publisher markers_pub;
image_transport::Publisher image_pub;

StereoBM sbm;
StereoSGBM sgbm;
Mat CameraMatrix1, CameraMatrix2, disCoeffs1, disCoeffs2, R1, R2, P1, P2, Q, T;


/**
   @brief Cameras connection
   @param[out] ppCameras
   @return void
 */
void ConnectCameras(Camera &Camera)
{
	//PointGrey

	Error error;

	BusManager busManager;
	unsigned int numCameras=0;
	while(numCameras!=1)
	{
		error = busManager.GetNumOfCameras(&numCameras);
		if(error != PGRERROR_OK)
		{
			cout<<"Failed to get number of cameras"<<endl;
			exit(1);
		}
		cout<<numCameras<<"cameras detected"<<endl;
	}

	//Camera camera1;

	PGRGuid pGuid;
	error = busManager.GetCameraFromIndex(0,&pGuid);
	if(error != PGRERROR_OK)
	{
		cout<<"Failed to get index of camera 1"<<endl;
		exit(1);
	}


	CameraInfo camInfo;

	//connect the two cameras
	error=Camera.Connect(&pGuid);
	if ( error != PGRERROR_OK )
	{
		cout << "Failed to connect to camera" <<endl;
		exit(1);
	}

	// Get the camera info and print it out
	error = Camera.GetCameraInfo( &camInfo );
	if ( error != PGRERROR_OK )
	{
		cout << "Failed to get camera info from camera" << endl;
		exit(1);
	}
	cout << camInfo.vendorName << " "
	     << camInfo.modelName << " "
	     << camInfo.serialNumber << endl;


	SetConfiguration(Camera);


	error = Camera.StartCapture();
	if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
	{
		cout << "Bandwidth exceeded" << endl;
		exit(1);
	}
	else if ( error != PGRERROR_OK )
	{
		cout << "Failed to start image capture" << endl;
		exit(1);
	}
}

/**
   @brief Image capture
   @param[in] ppCameras
   @return void
 */
void ImageCapture(Camera &Camera)
{
	Mat img;
	Mat imgHSV;
	Mat unImg;
	Mat imgBinary;

	// create the main window, and attach the trackbars
	namedWindow( "Camera", CV_WINDOW_NORMAL );
	namedWindow( "Binarized Image", CV_WINDOW_NORMAL );
	namedWindow( "Control", CV_WINDOW_NORMAL );
	namedWindow( "Circle", CV_WINDOW_NORMAL );
	namedWindow( "Canny", CV_WINDOW_NORMAL );

	/* Trackbars for Hue, Saturation and Value (HSV) in "Camera 1" window */
	int lowH = 0;
	int highH = 179;

	int lowS = 101;
	int highS = 255;

	int lowV = 37;
	int highV = 255;

	int valC = 200;

	int valA = 150;

	int maxR = 300;
	int minR = 150;

	// Hue 0-179
	cvCreateTrackbar("Upper Hue        ", "Control", &highH, 179);
	cvCreateTrackbar("Lower Hue        ", "Control", &lowH, 179);
	// Saturation 0-255
	cvCreateTrackbar("Upper Saturation", "Control", &highS, 255);
	cvCreateTrackbar("Lower Saturation", "Control", &lowS, 255);
	// Value 0-255
	cvCreateTrackbar("Upper Value     ", "Control", &highV, 255);
	cvCreateTrackbar("Lower Value     ", "Control", &lowV, 255);

	cvCreateTrackbar("Canny Threshold ", "Control", &valC, 200);

	// =========================================================================
	// Uncomment below if using Hough Circles
	// =========================================================================
	// cvCreateTrackbar("Accumulator     ", "Control", &valA, 255);
	//
	// cvCreateTrackbar("Max. Radius (p) ", "Control", &maxR, 720);
	// cvCreateTrackbar("Min. Radius (p) ", "Control", &minR, 720);
	// =========================================================================
	// Uncomment above if using Hough Circles
	// =========================================================================


	Property properties;
	properties.type=FRAME_RATE;
	Camera.GetProperty(&properties);
	cout<<"Frame rate "<<properties.absValue<<endl;

	sensor_msgs::ImagePtr image_msg;

	// capture loop
	int key = 0;
	while(key != 1048689 && key != 1114193)
	{
		// Get the image camera 1
		Image rawImage;
		Error error = Camera.RetrieveBuffer( &rawImage );
		if ( error != PGRERROR_OK )
		{
			cout << "capture error 1" << endl;
			continue;
		}

		// convert to rgb
		Image rgbImage;
		rawImage.Convert(PIXEL_FORMAT_BGR, &rgbImage );

		// convert to OpenCV Mat
		unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
		img = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

		imshow("Camera", img); // untouched image


		// =========================================================================
		// Pre-processing
		// =========================================================================

		// Convert the captured image frame BGR to HSV
		undistort(img,unImg,CameraMatrix1,disCoeffs1);
		cvtColor(unImg, imgHSV, COLOR_BGR2HSV);

		// Threshold the image
		inRange(imgHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), imgBinary);

		//morphological closing (fill small holes in the foreground)
		dilate( imgBinary, imgBinary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		erode(imgBinary, imgBinary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//morphological opening (remove small objects from the foreground)
		erode(imgBinary, imgBinary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgBinary, imgBinary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		GaussianBlur( imgBinary, imgBinary, Size(3, 3),2, 2 );

		imshow("Binarized Image", imgBinary);

		// =====================================================================
		// Circle detection
		// Hough Circles or Polygonal Curve fitting algorithm, pick one
		// =====================================================================

		//HoughCircles(unImg, imgBinary, valC, valA, minRadius, maxRadius);

		PolygonalCurveDetection(unImg, imgBinary, valC);

		if(!img.empty()) {
		 	image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
		 	image_pub.publish(image_msg);
		}

		// get user key
		key = waitKey(1);
		ros::spinOnce();
	}

	destroyWindow("Camera"); //destroy the window
	destroyWindow("Binarized Image");
	destroyWindow("Control");
	destroyWindow("Circle");
}


void HoughDetection(const Mat &img, const Mat& imgBinary, int valCanny, int valAccumulator, int minRadius, int maxRadius)
{
	Mat dst = img.clone();

	vector<Vec3f> circles;

	HoughCircles( imgBinary, circles, CV_HOUGH_GRADIENT, 1, imgBinary.rows/8, valCanny, valAccumulator, minRadius, maxRadius );

	for( size_t i = 0; i < circles.size(); i++ )
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		std::cout << "HoughCircles Radius = " <<radius << std::endl;
		// circle center
		circle( dst, center, 3, Scalar(255,0,0), -1, 8, 0 );
		// circle outline
		circle( dst, center, radius, Scalar(0,255,0), 1, 8, 0 );
		double Dist = CameraMatrix1.at<double>(0,0)*(BALL_DIAMETER/(2*radius));
		std::cout << "Real world diameter = " << Dist << std::endl;
	}

	imshow("Circles", dst);
}



/**
   @brief Ball detection in the stereo system
   @param[in]
   @return void
 */
//void ballDetection(sensor_msgs::PointCloud cloud)
void PolygonalCurveDetection( Mat &img, Mat &imgBinary, int valCanny )
{
	vector<vector<Point> > contours;

	/// Detect edges using canny
	Mat imgCanny;
	Canny( imgBinary, imgCanny, valCanny, valCanny*2, 3 );

	imshow("Canny", imgCanny); // debug

	findContours(imgCanny.clone(),contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	vector<Point> approx;
	Mat dst = img.clone();
	vector<double> areas;

	pcl::PointXYZ centroid; // Point structure for centroid
	/* The following lines make sure that position (0,0,0) is published when
	   the ball is not detected (avoids publishing previous positions when the
	   ball is no longer detected) */
	centroid.x = 0;
	centroid.y = 0;
	centroid.z = 0;

	pcl::PointXYZ centroidRadius; // Point structure for centroid
	/* The following lines make sure that position (0,0,0) is published when
	   the ball is not detected (avoids publishing previous positions when the
	   ball is no longer detected) */
	centroidRadius.x = 0;
	centroidRadius.y = 0;
	centroidRadius.z = 0;

	for(int i=0; i<contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02,true);

		// Skip small or non-convex objects
		if (std::fabs(cv::contourArea(contours[i])) < 1000 || !cv::isContourConvex(approx))
			continue;


		// Detect and label circles
		if(approx.size()>=6)
		{
			double area = cv::contourArea(contours[i]);
			cv::Rect r = cv::boundingRect(contours[i]);
			int radius = (r.width/2 +r.width/2)/2;

			if (abs(1 - ((double)r.width / r.height)) <= 0.2 && abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2
			    && area>1000)
			{
				//cout<< "radius - "<< radius <<endl; // DEBUGGING

				// Computing distnace from camera to center of detected circle
				double Dist;
				// + BALL_DIAMETER/4 is a correction since the sphere centroid is deeper than the center of the detected circle
				Dist = (CameraMatrix1.at<double>(0,0)*(BALL_DIAMETER/(radius*2)));
				std::stringstream s;
				s<<Dist;
				string str = s.str();
				setLabel(dst, str, contours[i]);

				centroid.x = r.x + radius;
				centroid.y = r.y + radius;
				centroid.z = radius;

				// Method based on circle radius =======================================
				cv::Mat image_vector = (cv::Mat_<double>(3,1) << centroid.x, centroid.y, 1);
				cv::Mat camera_vector = CameraMatrix1;

				camera_vector = CameraMatrix1.inv() * (Dist * image_vector);
				//cout << camera_vector << endl; // DEBUGGING
				//cout << CameraMatrix1.inv() << endl; // DEBUGGING
				centroidRadius.x = camera_vector.at<double>(0);
				centroidRadius.y = camera_vector.at<double>(1);
				centroidRadius.z = camera_vector.at<double>(2);
				//cout << centroidRadius << endl; // DEBUGGING
			}
		}
	}
	CentroidPub(centroid, centroidRadius);
	imshow("Circle", dst);
}

void CentroidPub( const pcl::PointXYZ centroid, const pcl::PointXYZ centroidRadius )
{
	// Method based on solvePnP ================================================
	geometry_msgs::PointStamped CentroidCam;

	CentroidCam.point.x = centroid.x;
	CentroidCam.point.y = centroid.y;
	CentroidCam.point.z = centroid.z;

	CentroidCam.header.stamp = ros::Time::now();
	ballCentroidCamPnP_pub.publish(CentroidCam);
	//std::cout << CentroidCam << std::endl;

	// Method based on circle radius ===========================================
	CentroidCam.point.x = centroidRadius.x;
	CentroidCam.point.y = centroidRadius.y;
	CentroidCam.point.z = centroidRadius.z;

	CentroidCam.header.stamp = ros::Time::now();
	ballCentroidCam_pub.publish(CentroidCam);
	//std::cout << CentroidCam << std::endl;

	visualization_msgs::MarkerArray targets_markers;
	targets_markers.markers = createTargetMarkers(centroid);
	markers_pub.publish(targets_markers);
}

void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.8;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);
	int radius = (r.width/2 + r.height/2)/2;

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::Point center(r.x + ((r.width) / 2), r.y + ((r.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
	circle( im, center, 3, Scalar(255,0,0), -1, 8, 0 );
	// circle outline
	circle( im, center, radius, Scalar(0,255,0), 1, 8, 0 );
}

/**
   @brief Main function of the camera node
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Point_Grey");
	ros::NodeHandle n;

	//read calibration paraneters
	string a="/src/mystereocalib.yml";
	string path = ros::package::getPath("calibration");
	path=path+a;
	FileStorage fs(path, FileStorage::READ);
	if(!fs.isOpened())
	{
		cout<<"failed to open document"<<endl;
		return -1;
	}

	fs["CM1"] >> CameraMatrix1;
	fs["D1"] >> disCoeffs1;

	sbm.state->SADWindowSize = 7;
	sbm.state->numberOfDisparities = 112;
	sbm.state->preFilterSize = 5;
	sbm.state->preFilterCap = 61;
	sbm.state->minDisparity = 0;
	sbm.state->textureThreshold = 700;
	sbm.state->uniquenessRatio = 0;
	sbm.state->speckleWindowSize = 0;
	sbm.state->speckleRange = 8;
	sbm.state->disp12MaxDiff = 1;

	sgbm.SADWindowSize=5;
	sgbm.numberOfDisparities=192;
	sgbm.preFilterCap=4;
	sgbm.minDisparity=-64;
	sgbm.uniquenessRatio=1;
	sgbm.speckleWindowSize = 150;
	sgbm.speckleRange=2;
	sgbm.disp12MaxDiff=10;
	sgbm.fullDP=false;
	sgbm.P1=950;
	sgbm.P2=2500;

	image_transport::ImageTransport it(n);
	image_pub = it.advertise("/SingleCamera/image", 1000);
	ballCentroidCam_pub = n.advertise<geometry_msgs::PointStamped>( "/SingleCamera/ballCentroid", 7);
	ballCentroidCamPnP_pub = n.advertise<geometry_msgs::PointStamped>( "/SingleCamera/ballCentroidPnP", 7);
	markers_pub = n.advertise<visualization_msgs::MarkerArray>( "/markers4", 1000);

  Camera Camera;
	ConnectCameras(Camera);
	ImageCapture(Camera);

	return 0;
}



// /**
//    @brief Stereo 3D reconstraction
//    @param[in] image1 image from the first camera
//    @param[in] image2 image from the second camera
//    @return void
//  */
// void reconstrution3D(Mat image1, Mat image2)
// {
//      Mat map1x, map1y, map2x, map2y;
//      Mat imgU1, imgU2;
//
//      initUndistortRectifyMap(CameraMatrix1, disCoeffs1, R1, P1, image1.size(), CV_32FC1, map1x, map1y);
//      initUndistortRectifyMap(CameraMatrix2, disCoeffs2, R2, P2, image1.size(), CV_32FC1, map2x, map2y);
//
//      remap(image1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
//      remap(image2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
//
//      createTrackbar("P1", "Disparity", &sgbm.P1, 2000, NULL);
//      createTrackbar("P2", "Disparity", &sgbm.P2, 4000, NULL);
//      createTrackbar("preFilterCap", "Disparity", &sgbm.preFilterCap, 200, NULL);
//      createTrackbar("speckleWindowSize", "Disparity", &sgbm.speckleWindowSize, 200, NULL);
// }
//
// /**
//    @brief Create a point cloud from the 3D reconstruction
//    @param[in] XYZ Matrix with 3D points
//    @return void
//  */
// void CreatePointCloud(Mat XYZ)
// {
//      sensor_msgs::PointCloud cloud;
//      pcl::PointCloud<pcl::PointXYZ> SwissRanger_cloud;
//
//      for(int i; i<XYZ.rows; i++)
//      {
//              for(int j=0; j<XYZ.cols; j++)
//              {
//                      Point3f point = XYZ.at<Point3f>(i, j);
//                      geometry_msgs::Point32 point3D;
//                      if(point.z>0.2 && point.z<4)
//                      {
//                              point3D.x = point.x;
//                              point3D.y = point.y;
//                              point3D.z = point.z;
//                              cloud.points.push_back(point3D);
//
//                              pcl::PointXYZ p;
//                              p.x=cloud.points[i].x;
//                              p.y=cloud.points[i].y;
//                              p.z=cloud.points[i].z;
//                              SwissRanger_cloud.push_back(p);
//                      }
//              }
//      }
//      cout<<"size "<<cloud.points.size()<<endl;
//      if(cloud.points.size()>0)
//      {
//
//              ballDetection(cloud);
//      }
// }
//
// /**
//    @brief Ball detection in the stereo system
//    @param[in] cloud point cloud from the stereo system
//    @return void
//  */
// void ballDetection(sensor_msgs::PointCloud cloud)
// {
//      pcl::PointCloud<pcl::PointXYZ> camera_cloud;
//      for(int i; i<cloud.points.size(); i++)
//      {
//              pcl::PointXYZ p;
//              p.x=cloud.points[i].x;
//              p.y=cloud.points[i].y;
//              p.z=cloud.points[i].z;
//              camera_cloud.push_back(p);
//      }
//
//      pcl::PCLPointCloud2 cloud2;
//      pcl::toPCLPointCloud2 (camera_cloud, cloud2);
//      pcl::PCLPointCloud2::Ptr cloud3 (new pcl::PCLPointCloud2 (cloud2));
//      pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
//      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//      sor.setInputCloud (cloud3);
//      sor.setLeafSize (0.01f, 0.01f, 0.01f);
//      sor.filter (*cloud_filtered);
//
//      sensor_msgs::PointCloud2 pp;
//
//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(camera_cloud));
//
//      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//      pcl::SACSegmentation<pcl::PointXYZ> seg;
//      seg.setOptimizeCoefficients (true);
//      seg.setModelType (pcl::SACMODEL_SPHERE); //detecting SPHERE
//      seg.setMethodType (pcl::SAC_RANSAC);
//      seg.setDistanceThreshold (0.02);
//      seg.setRadiusLimits(0.35, 0.55);
//      seg.setMaxIterations(1000000);
//      seg.setInputCloud (cloudPtr);
//      seg.segment (*inliers, *coefficients);
//
//      if(inliers->indices.size ()>0)
//      {
//              cout << "Model coefficients: " << coefficients->values[0] << " "
//                   << coefficients->values[1] << " "
//                   << coefficients->values[2] << " "
//                   << coefficients->values[3] << endl;
//
//              pcl::PointXYZ center;
//              center.x = coefficients->values[0];
//              center.y = coefficients->values[1];
//              center.z = coefficients->values[2];
//
//              geometry_msgs::PointStamped CentroidCam_1, CentroidCam_2;
//
//              if(coefficients->values[3]<0.6)
//              {
//                      CentroidCam_1.point.x = coefficients->values[0];
//                      CentroidCam_1.point.y = coefficients->values[1];
//                      CentroidCam_1.point.z = coefficients->values[2];
//                      CentroidCam_1.header.stamp = ros::Time::now();
//                      ballCentroidCam1_pub.publish(CentroidCam_1);
//
//                      CentroidCam_2.point.x = coefficients->values[0]+T.at<double>(0,0)*pow(10,-1);
//                      CentroidCam_2.point.y = coefficients->values[1];
//                      CentroidCam_2.point.z = coefficients->values[2];
//                      CentroidCam_2.header.stamp = ros::Time::now();
//                      ballCentroidCam2_pub.publish(CentroidCam_2);
//              }
//              else
//              {
//                      CentroidCam_1.point.x = 0;
//                      CentroidCam_1.point.y = 0;
//                      CentroidCam_1.point.z = 0;
//                      CentroidCam_1.header.stamp = ros::Time::now();
//                      ballCentroidCam1_pub.publish(CentroidCam_1);
//
//                      CentroidCam_2.point.x = 0;
//                      CentroidCam_2.point.y = 0;
//                      CentroidCam_2.point.z = 0;
//                      CentroidCam_2.header.stamp = ros::Time::now();
//                      ballCentroidCam1_pub.publish(CentroidCam_2);
//              }
//
//              visualization_msgs::MarkerArray targets_markers;
//              targets_markers.markers = createTargetMarkers(center);
//              markers_pub.publish(targets_markers);
//      }
//
//
//      pcl::toROSMsg(*cloudPtr,pp);
//      sensor_msgs::PointCloud stereoCloud;
//      sensor_msgs::convertPointCloud2ToPointCloud(pp, stereoCloud);
//
//      stereoCloud.header.frame_id="/my_frame";
//      stereoCloud.header.stamp=ros::Time::now();
//      stereoCloud_pub.publish(stereoCloud);
//
// }
