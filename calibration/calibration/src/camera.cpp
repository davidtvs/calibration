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
 \file  camera.cpp
 \brief Ball detection with a stereo system
 \author Marcelo Pereira
 \date   December, 2015
*/

#include "calibration/camera_config.h"
#include <ros/package.h>

//Marker's publisher
ros::Publisher ballCentroidCam1_pub;
ros::Publisher ballCentroidCam2_pub;
ros::Publisher stereoCloud_pub;
ros::Publisher markers_pub;

StereoBM sbm;
StereoSGBM sgbm;
Mat CameraMatrix1, CameraMatrix2, disCoeffs1, disCoeffs2, R1, R2, P1, P2, Q, T;

void HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold, int numCamera)
    {

        // will hold the results of the detection
        vector<Vec3f> circles;
        // runs the actual detection
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, cannyThreshold, accumulatorThreshold, 110, 195 );


        // clone the colour, input image for displaying purposes
        Mat display = src_display.clone();
        for( size_t i = 0; i < circles.size(); i++ )
        {
            cout<<"camera "<<numCamera<<endl;
            cout<<"radius"<<circles[i][2]<<endl;
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            cout<<"center x"<<circles[i][0]<<" y "<<circles[i][1]<<endl;
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }

        // shows the results
        if(numCamera==1)
            imshow( "Hough Circle Transform", display);
        else
            imshow( "Hough Circle Transform 2", display);
    }

/**
@brief Cameras connection
@param[out] ppCameras
@return void
*/
void ConnectCameras(Camera** &ppCameras)
{
    //PointGrey

    Error error;

    BusManager busManager;
    unsigned int numCameras=0;
    while(numCameras!=2)
    {
        error = busManager.GetNumOfCameras(&numCameras);
        if(error != PGRERROR_OK)
        {
            cout<<"Failed to get number of cameras"<<endl;
            exit(1);
        }
        cout<<numCameras<<"cameras detected"<<endl;
    }

    //Camera camera1, camera2;
    ppCameras = new Camera*[numCameras];

    for(unsigned int i=0; i<numCameras;i++)
    {
        ppCameras[i] = new Camera();

        PGRGuid pGuid;
        error = busManager.GetCameraFromIndex(i,&pGuid);
        if(error != PGRERROR_OK)
        {
            cout<<"Failed to get index of camera 1"<<endl;
            exit(1);
        }


        CameraInfo camInfo;

        //connect the two cameras
        error=ppCameras[i]->Connect(&pGuid);
        if ( error != PGRERROR_OK )
        {
            cout << "Failed to connect to camera"<<i <<endl;
            exit(1);
        }

        // Get the camera info and print it out
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        if ( error != PGRERROR_OK )
        {
            cout << "Failed to get camera info from camera" << endl;
            exit(1);
        }
        cout << camInfo.vendorName << " "
                  << camInfo.modelName << " "
                  << camInfo.serialNumber << endl;
    }

    //SetConfiguration(ppCameras[0]);
    //SetConfiguration(ppCameras[1]);

    for(int i =0;i<numCameras;i++)
    {
        error = ppCameras[i]->StartCapture();
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
}

/**
@brief Image capture
@param[in] ppCameras
@return void
*/
void ImageCapture(Camera** ppCameras)
{
    Mat imgGRAY;
    Mat image1, image2;

    // create the main window, and attach the trackbars
    namedWindow( "Camera 1", CV_WINDOW_NORMAL );
    namedWindow( "Camera 2", CV_WINDOW_NORMAL );
    namedWindow( "Disparity", CV_WINDOW_NORMAL );
    //namedWindow( "Disparity sbm", CV_WINDOW_NORMAL );

    Property properties;
    properties.type=FRAME_RATE;
    ppCameras[0]->GetProperty(&properties);
    cout<<"Frame rate "<<properties.absValue<<endl;

    // capture loop
    int key = 0;
    while(key != 1048689 && key != 1114193)
    {
        // Get the image camera 1
        Image rawImage;
        Error error = ppCameras[0]->RetrieveBuffer( &rawImage );
        if ( error != PGRERROR_OK )
        {
            //cout << "capture error" << endl;
            continue;
        }

        // convert to rgb
        Image rgbImage;
        rawImage.Convert(PIXEL_FORMAT_BGR, &rgbImage );

        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
        image1 = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);


        // Get the image camera 2
        Image rawImage2;
        error = ppCameras[1]->RetrieveBuffer( &rawImage2 );
        if ( error != PGRERROR_OK )
        {
            //cout << "capture error" << endl;
            continue;
        }

        // convert to rgb
        Image rgbImage2;
        rawImage2.Convert(PIXEL_FORMAT_BGR, &rgbImage2 );

        // convert to OpenCV Mat
        unsigned int rowBytes2 = (double)rgbImage2.GetReceivedDataSize()/(double)rgbImage2.GetRows();
        image2 = Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes2);

        imshow("Camera 1", image1);
        imshow("Camera 2", image2);

        Mat img1, img2;
        img1=image1;
        img2=image2;


        cvtColor(img1, image1, CV_BGR2GRAY);//convert the image from BGR to Gray scale
        cvtColor(img2, image2, CV_BGR2GRAY);//convert the image from BGR to Gray scale

        reconstrution3D(image1,image2);

        // get user key
        key = waitKey(5);
        ros::spinOnce();
    }

    destroyWindow("Camera 1"); //destroy the window
    destroyWindow("Camera 2");
    destroyWindow("Disparity");
}

/**
@brief Stereo 3D reconstraction
@param[in] image1 image from the first camera
@param[in] image2 image from the second camera
@return void
*/
void reconstrution3D(Mat image1, Mat image2)
{
    Mat map1x, map1y, map2x, map2y;
    Mat imgU1, imgU2;

    initUndistortRectifyMap(CameraMatrix1, disCoeffs1, R1, P1, image1.size(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(CameraMatrix2, disCoeffs2, R2, P2, image1.size(), CV_32FC1, map2x, map2y);

    remap(image1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
    remap(image2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

    Mat disp, disp8;

    // Compute disparity
    sgbm(imgU1, imgU2,disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    imshow("Disparity", disp8);

    // Compute world coordinates from the disparity image
    Mat XYZ(disp8.size(),CV_32FC3);
    reprojectImageTo3D(disp, XYZ, Q, false, CV_32F);
    CreatePointCloud(XYZ);

    createTrackbar("P1", "Disparity", &sgbm.P1, 2000, NULL);
    createTrackbar("P2", "Disparity", &sgbm.P2, 4000, NULL);
    createTrackbar("preFilterCap", "Disparity", &sgbm.preFilterCap, 200, NULL);
    createTrackbar("speckleWindowSize", "Disparity", &sgbm.speckleWindowSize, 200, NULL);

}

/**
@brief Create a point cloud from the 3D reconstruction
@param[in] XYZ Matrix with 3D points
@return void
*/
void CreatePointCloud(Mat XYZ)
{
    sensor_msgs::PointCloud cloud;
    pcl::PointCloud<pcl::PointXYZ> SwissRanger_cloud;

    for(int i;i<XYZ.rows;i++)
    {
        for(int j=0;j<XYZ.cols;j++)
        {
            Point3f point = XYZ.at<Point3f>(i, j);
            geometry_msgs::Point32 point3D;
            if(point.z>0.2 && point.z<4)
            {
                point3D.x = point.x;
                point3D.y = point.y;
                point3D.z = point.z;
                cloud.points.push_back(point3D);

                pcl::PointXYZ p;
                p.x=cloud.points[i].x;
                p.y=cloud.points[i].y;
                p.z=cloud.points[i].z;
                SwissRanger_cloud.push_back(p);
            }
        }
    }
    cout<<"size "<<cloud.points.size()<<endl;
    if(cloud.points.size()>0)
    {

        ballDetection(cloud);
    }
}

/**
@brief Ball detection in the stereo system
@param[in] cloud point cloud from the stereo system
@return void
*/
void ballDetection(sensor_msgs::PointCloud cloud)
{
    pcl::PointCloud<pcl::PointXYZ> camera_cloud;
    for(int i;i<cloud.points.size();i++)
    {
        pcl::PointXYZ p;
        p.x=cloud.points[i].x;
        p.y=cloud.points[i].y;
        p.z=cloud.points[i].z;
        camera_cloud.push_back(p);
    }

    pcl::PCLPointCloud2 cloud2;
    pcl::toPCLPointCloud2 (camera_cloud, cloud2);
    pcl::PCLPointCloud2::Ptr cloud3 (new pcl::PCLPointCloud2 (cloud2));
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud3);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    sensor_msgs::PointCloud2 pp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(camera_cloud));

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_SPHERE); //detecting SPHERE
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);
    seg.setRadiusLimits(0.35, 0.55);
    seg.setMaxIterations(1000000);
    seg.setInputCloud (cloudPtr);
    seg.segment (*inliers, *coefficients);

    if(inliers->indices.size ()>0)
    {
        cout << "Model coefficients: " << coefficients->values[0] << " "
             << coefficients->values[1] << " "
             << coefficients->values[2] << " "
             << coefficients->values[3] << endl;

        pcl::PointXYZ center;
        center.x = coefficients->values[0];
        center.y = coefficients->values[1];
        center.z = coefficients->values[2];

        geometry_msgs::PointStamped CentroidCam_1, CentroidCam_2;

        if(coefficients->values[3]<0.6)
        {
            CentroidCam_1.point.x = coefficients->values[0];
            CentroidCam_1.point.y = coefficients->values[1];
            CentroidCam_1.point.z = coefficients->values[2];
            CentroidCam_1.header.stamp = ros::Time::now();
            ballCentroidCam1_pub.publish(CentroidCam_1);

            CentroidCam_2.point.x = coefficients->values[0]+T.at<double>(0,0)*pow(10,-1);
            CentroidCam_2.point.y = coefficients->values[1];
            CentroidCam_2.point.z = coefficients->values[2];
            CentroidCam_2.header.stamp = ros::Time::now();
            ballCentroidCam2_pub.publish(CentroidCam_2);
        }
        else
        {
            CentroidCam_1.point.x = 0;
            CentroidCam_1.point.y = 0;
            CentroidCam_1.point.z = 0;
            CentroidCam_1.header.stamp = ros::Time::now();
            ballCentroidCam1_pub.publish(CentroidCam_1);

            CentroidCam_2.point.x = 0;
            CentroidCam_2.point.y = 0;
            CentroidCam_2.point.z = 0;
            CentroidCam_2.header.stamp = ros::Time::now();
            ballCentroidCam1_pub.publish(CentroidCam_2);
        }

        visualization_msgs::MarkerArray targets_markers;
        targets_markers.markers = createTargetMarkers(center);
        markers_pub.publish(targets_markers);
    }


    pcl::toROSMsg(*cloudPtr,pp);
    sensor_msgs::PointCloud stereoCloud;
    sensor_msgs::convertPointCloud2ToPointCloud(pp, stereoCloud);

    stereoCloud.header.frame_id="/my_frame";
    stereoCloud.header.stamp=ros::Time::now();
    stereoCloud_pub.publish(stereoCloud);

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
    fs["CM2"] >> CameraMatrix2;
    fs["D1"] >> disCoeffs1;
    fs["D2"] >> disCoeffs2;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;
    fs["T"] >>T;

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

    ballCentroidCam1_pub = n.advertise<geometry_msgs::PointStamped>( "/Camera1/ballCentroid", 10000);
    ballCentroidCam2_pub = n.advertise<geometry_msgs::PointStamped>( "/Camera2/ballCentroid", 10000);
    stereoCloud_pub=n.advertise<sensor_msgs::PointCloud>("stereoCloud", 10000);
    markers_pub = n.advertise<visualization_msgs::MarkerArray>( "/markers4", 10000);

    Camera** ppCameras;
    ConnectCameras(ppCameras);
    ImageCapture(ppCameras);

    return 0;
}
