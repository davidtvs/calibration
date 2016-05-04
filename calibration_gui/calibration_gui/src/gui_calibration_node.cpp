/*****************************************************************************
** Includes
*****************************************************************************/
#define _NODE_CPP_

// ROS includes
#include <ros/ros.h>
#include <ros/network.h>

// Project includes
#include "calibration_gui/gui_calibration_node.hpp"
#include "calibration_gui/gui_mainwindow.h"
#include "ui_mainwindow.h"
#include "calibration_gui/calibration.h"
#include "calibration_gui/visualization_rviz_calibration.h"

// Generic includes
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <QDebug>

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv, const std::string &name ) :
    init_argc(argc),
    init_argv(argv),
    node_name(name)
{}

QNode::~QNode() {
    shutdown();
}
/**
 * This is called by the qt application to stop the ros node before the
 * qt app closes.
 */
void QNode::shutdown() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::on_init() {
    ros::init(init_argc,init_argv,node_name);
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
    //start();
    return true;
}

bool QNode::on_init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,node_name);
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
    //start();
    return true;
}

void QNode::run() {
    ros::Publisher markers_pub;
    ros::Publisher car_pub;
    pcl::PointCloud<pcl::PointXYZ> lms1PointCloud;
    pcl::PointCloud<pcl::PointXYZ> lms2PointCloud;
    pcl::PointCloud<pcl::PointXYZ> ldmrPointCloud;
    pcl::PointCloud<pcl::PointXYZ> swissrangerCloud;
    pcl::PointCloud<pcl::PointXYZ> camera1Cloud;
    pcl::PointCloud<pcl::PointXYZ> singleCamCloud;
    pcl::PointCloud<pcl::PointXYZ> singleCamCloudPnP;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Lms1PointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Lms2PointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr LdmrPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Camera1Cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr SingleCamCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr SingleCamCloudPnP (new pcl::PointCloud<pcl::PointXYZ>);
    geometry_msgs::Pose laser_ldmrs;
    geometry_msgs::Pose laser_lms151_1;
    geometry_msgs::Pose laser_lms151_2;
    geometry_msgs::Pose swissranger;
    geometry_msgs::Pose stereo;
    geometry_msgs::Pose singleCam;
    geometry_msgs::Pose singleCamPnP;
    geometry_msgs::Pose singleCamPnPRansac;


    ros::NodeHandle n;
    // Get number of points to collect from TreeWidget
    // Should be on cell (1,1), but just in case this is changed the code below searches for the string in the whole tree
    //int num_of_points = MainWindow::treeWidgetFindText( "Number of Calibration Points" );
    qDebug() << num_of_points;


    markers_pub = n.advertise<visualization_msgs::MarkerArray>( "/markers3", 10000);
    car_pub = n.advertise<visualization_msgs::Marker>("/ATLASCAR1", 1);

    CircleCentroids centroids;
    int count=0, change=0;
    pcl::PointXYZ P[2];
    P[1].x=0;
    P[1].y=0;
    P[1].z=0;

    ldmrPointCloud.clear();
    lms1PointCloud.clear();
    lms2PointCloud.clear();
    swissrangerCloud.clear();
    camera1Cloud.clear();
    singleCamCloud.clear();
    singleCamCloudPnP.clear();

    laser_lms151_1.orientation.w=1.0; // so it can be multiplied by transformations later

    vector<cv::Point3f> objectPoints;
    vector<cv::Point2f> imagePoints;

    createDirectory( );

    const char* filename_obj_img = "singleCam_obj_img_points.txt"; // file filename that stores object points and image points
    string tmp_str = file_path + filename_obj_img;
    char* FilePath_obj_img = new char [tmp_str.length() + 1];
    strcpy (FilePath_obj_img, tmp_str.c_str());

    FILE* pFile = fopen(FilePath_obj_img, "w");
    fprintf(pFile, "ObjectPoints (m)\tImagePoints (pixel)\n");
    fprintf(pFile, "x;y;z\tx;y;r\n");
    fclose(pFile);

    qDebug() << "Calibration is going to start";

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        while(count < num_of_points)
        {
            //cout << "calibration: test\n" << "lms1 - " << centroids.lms1Centroid.point.x << " | lms2 - " <<  centroids.lms2Centroid.point.x << endl; // debug
            //cout<<"cycle start"<<endl;
            //cout << "(" << centroids.singleCamCentroid.point.x << "," << centroids.singleCamCentroid.point.y << "," <<  centroids.singleCamCentroid.point.z << ")" << endl;
            if(centroids.lms1Centroid.point.x != 0 && centroids.lms2Centroid.point.x != 0
                    && centroids.ldmrsCentroid.point.x != 0 && centroids.singleCamCentroid.point.z != 0
                    && centroids.singleCamCentroidPnP.point.z != 0)        //&& centroids.cam1Centroid.point.z!=0)
            {
                P[0].x=centroids.lms1Centroid.point.x;
                P[0].y=centroids.lms1Centroid.point.y;
                P[0].z=centroids.lms1Centroid.point.z;

                double dist;
                dist=sqrt(pow((P[0].x-P[1].x),2) + pow((P[0].y-P[1].y),2));
                double time_diference;
                time_diference = centroids.lms2Centroid.header.stamp.nsec - centroids.lms1Centroid.header.stamp.nsec;
                cout<<"time = "<<time_diference<<endl;
                cout<<"dist = "<<dist<<endl;
                if(dist>0.10)
                {
                    double diff_dist_mean=0;
                    if(count>0)
                    {
                        double d_ldmrs, d_lms1, d_lms2, d_sr, d_stereo;
                        // calculation of the distance between the corrent center of the ball detected and the previous
                        d_ldmrs=sqrt(pow((ldmrPointCloud.points[count-1].x-centroids.ldmrsCentroid.point.x),2)+pow((ldmrPointCloud.points[count-1].y-centroids.ldmrsCentroid.point.y),2)+pow((ldmrPointCloud.points[count-1].z-centroids.ldmrsCentroid.point.z),2));
                        d_lms1=sqrt(pow((lms1PointCloud.points[count-1].x-centroids.lms1Centroid.point.x),2)+pow((lms1PointCloud.points[count-1].y-centroids.lms1Centroid.point.y),2)+pow((lms1PointCloud.points[count-1].z-centroids.lms1Centroid.point.z),2));
                        d_lms2=sqrt(pow((lms2PointCloud.points[count-1].x-centroids.lms2Centroid.point.x),2)+pow((lms2PointCloud.points[count-1].y-centroids.lms2Centroid.point.y),2)+pow((lms2PointCloud.points[count-1].z-centroids.lms2Centroid.point.z),2));
                        // d_sr=sqrt(pow((swissrangerCloud.points[count-1].x-centroids.swissrangerCentroid.point.x),2)+pow((swissrangerCloud.points[count-1].y-centroids.swissrangerCentroid.point.y),2)+pow((swissrangerCloud.points[count-1].z-centroids.swissrangerCentroid.point.z),2));
                        // d_stereo=sqrt(pow((camera1Cloud.points[count-1].x-centroids.cam1Centroid.point.x),2)+pow((camera1Cloud.points[count-1].y-centroids.cam1Centroid.point.y),2)+pow((camera1Cloud.points[count-1].z-centroids.cam1Centroid.point.z),2));
                        int number_of_sensors = 2;         //number of sensors to be calibrated (without counting the reference sensor)
                        // diff_dist_mean=(pow((d_lms1-d_ldmrs),2)+pow((d_lms1-d_stereo),2)+pow((d_lms1-d_lms2),2))/number_of_sensors;
                        // diff_dist_mean=(pow((d_lms1-d_lms2),2))/number_of_sensors; //teste com 2 LMS151
                        diff_dist_mean=(pow((d_lms1-d_ldmrs),2)+pow((d_lms1-d_lms2),2))/number_of_sensors;
                    }
                    std::cout << "diff_dist_mean = " << diff_dist_mean << std::endl;
                    if(diff_dist_mean<0.15)    // Valid point     // ?? Why less and not greater?
                    {
                        pcl::PointXYZ p;
                        p.x=centroids.lms1Centroid.point.x;
                        p.y=centroids.lms1Centroid.point.y;
                        p.z=centroids.lms1Centroid.point.z;
                        lms1PointCloud.push_back(p);

                        objectPoints.push_back(cv::Point3f(p.x, p.y, p.z));
                        cout << "objectPoint = " << p << endl; // prints current objectPoint


                        pFile = fopen(FilePath_obj_img, "a");
                        fprintf(pFile, "%F;%F;%F\t", p.x, p.y, p.z);

                        p.x=centroids.lms2Centroid.point.x;
                        p.y=centroids.lms2Centroid.point.y;
                        p.z=centroids.lms2Centroid.point.z;
                        lms2PointCloud.push_back(p);

                        p.x=centroids.ldmrsCentroid.point.x;
                        p.y=centroids.ldmrsCentroid.point.y;
                        p.z=centroids.ldmrsCentroid.point.z;
                        ldmrPointCloud.push_back(p);

                        // p.x=centroids.swissrangerCentroid.point.x;
                        // p.y=centroids.swissrangerCentroid.point.y;
                        // p.z=centroids.swissrangerCentroid.point.z;
                        // swissrangerCloud.push_back(p);
                        //
                        // p.x=centroids.cam1Centroid.point.x;
                        // p.y=centroids.cam1Centroid.point.y;
                        // p.z=centroids.cam1Centroid.point.z;
                        // camera1Cloud.push_back(p);

                        // PointCLoud used on radius based method
                        p.x=centroids.singleCamCentroid.point.x;
                        p.y=centroids.singleCamCentroid.point.y;
                        p.z=centroids.singleCamCentroid.point.z;
                        singleCamCloud.push_back(p);
                        cout << "singleCamCloud = " << p << endl; // prints current Camera point

                        // Image points used on solvePnP based method
                        p.x = centroids.singleCamCentroidPnP.point.x;
                        p.y = centroids.singleCamCentroidPnP.point.y;
                        p.z = centroids.singleCamCentroidPnP.point.z;
                        imagePoints.push_back(cv::Point2f(p.x, p.y));
                        singleCamCloudPnP.push_back(p); // Point cloud with imagePoints so I can save to PCD file
                        cout << "imagePoint = " << p << endl; // prints current imagePoint

                        // save imagePoints to file
                        fprintf(pFile, "%F;%F;%F\n", p.x, p.y, p.z);
                        fclose(pFile);

                        cout<<"count "<<count+1<<endl;

                        cout<<"Press to select another point";
                        cin.ignore();
                        count++;
                        change=0;
                    }

                    // Storing point clouds in clouds
                    vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
                    clouds.push_back(lms1PointCloud);
                    clouds.push_back(lms2PointCloud);
                    clouds.push_back(ldmrPointCloud);
                    //clouds.push_back(swissrangerCloud);
                    //clouds.push_back(camera1Cloud);
                    clouds.push_back(singleCamCloud);
                    clouds.push_back(singleCamCloudPnP); // for SolvePnP
                    clouds.push_back(singleCamCloudPnP); // for SolvePnPRansac

                    // Saving all point clouds to a PCD file
                    pcl::io::savePCDFileASCII(file_path + "lms1.pcd", lms1PointCloud);
                    pcl::io::savePCDFileASCII(file_path + "lms2.pcd", lms2PointCloud);
                    pcl::io::savePCDFileASCII(file_path + "ldmrs.pcd", ldmrPointCloud);
                    //pcl::io::savePCDFileASCII("swissranger.pcd", swissrangerCloud);
                    //pcl::io::savePCDFileASCII("stereo.pcd", camera1Cloud);
                    pcl::io::savePCDFileASCII(file_path + "singleCamCloud.pcd", singleCamCloud);
                    pcl::io::savePCDFileASCII(file_path + "singleCamCloudPnP.pcd", singleCamCloudPnP); // imagePoints and radius cloud

                    string imgPath = file_path + "img_" + boost::lexical_cast<std::string>(count) + ".jpg";
                    cv::Mat img;
                    img = cv_bridge::toCvShare(centroids.camImage, "bgr8")->image;
                    imwrite( imgPath, img );

                    // Sets pose
                    vector<geometry_msgs::Pose> lasers;
                    lasers.push_back(laser_lms151_1);
                    lasers.push_back(laser_lms151_2);
                    lasers.push_back(laser_ldmrs);
                    //lasers.push_back(swissranger);
                    //lasers.push_back(stereo);
                    lasers.push_back(singleCam);
                    lasers.push_back(singleCamPnP);
                    lasers.push_back(singleCamPnPRansac);

                    visualization_msgs::MarkerArray targets_markers;
                    targets_markers.markers = createTargetMarkers(clouds,lasers);
                    markers_pub.publish(targets_markers);
                }

                if(change==0)
                {
                    P[1].x=centroids.lms1Centroid.point.x;
                    P[1].y=centroids.lms1Centroid.point.y;
                    P[1].z=centroids.lms1Centroid.point.z;
                    change=1;
                }
            }
            ros::spinOnce();
            loop_rate.sleep();
        }

        cout<<"Press Enter to Continue";
        cin.ignore();

        // What happens below????
        vector<pcl::PointCloud<pcl::PointXYZ> > cloudss;
        cloudss.push_back(lms1PointCloud);
        cloudss.push_back(lms2PointCloud);
        cloudss.push_back(ldmrPointCloud);
        //cloudss.push_back(swissrangerCloud);
        //cloudss.push_back(camera1Cloud);
        cloudss.push_back(singleCamCloud);
        cloudss.push_back(singleCamCloudPnP); // for SolvePnP
        cloudss.push_back(singleCamCloudPnP); // for SolvePnPRansac

        vector<geometry_msgs::Pose> laserss;
        laserss.push_back(laser_lms151_1);
        laserss.push_back(laser_lms151_2);
        laserss.push_back(laser_ldmrs);
        //laserss.push_back(swissranger);
        //laserss.push_back(stereo);
        laserss.push_back(singleCam);
        laserss.push_back(singleCamPnP);
        laserss.push_back(singleCamPnPRansac);

        visualization_msgs::MarkerArray targets_markerss;
        targets_markerss.markers = createTargetMarkers(cloudss,laserss);
        markers_pub.publish(targets_markerss);

        estimateTransformation(laser_lms151_2,lms1PointCloud,lms2PointCloud, "lms1_lms2");
        estimateTransformation(laser_ldmrs,lms1PointCloud,ldmrPointCloud, "lms1_ldmrs");
        // estimateTransformation(swissranger,lmsPointCloud,swissrangerCloud,"lms_sr");
        // estimateTransformation(stereo,lms1PointCloud,camera1Cloud,"lms1_stereo");
        estimateTransformation(singleCam, lms1PointCloud, singleCamCloud, "lms1_camera");
        estimateTransformationCamera(singleCamPnP, objectPoints, imagePoints, "lms1_camera_calib", true, false); // Transformation estimation with solvePnP
        estimateTransformationCamera(singleCamPnPRansac, objectPoints, imagePoints, "lms1_camera_calib", true, true); // Transformation estimation with solvePnPRansac

        cout<<"end"<<endl;

        cout<<"Press Enter to Continue";
        cin.ignore();

        vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
        clouds.push_back(lms1PointCloud);
        clouds.push_back(lms2PointCloud);
        clouds.push_back(ldmrPointCloud);
        //clouds.push_back(swissrangerCloud);
        //clouds.push_back(camera1Cloud);
        clouds.push_back(singleCamCloud);
        clouds.push_back(singleCamCloudPnP); // for SolvePnP
        clouds.push_back(singleCamCloudPnP); // for SolvePnPRansac

        // Sets pose according to estimated transformations
        vector<geometry_msgs::Pose> lasers;
        lasers.push_back(laser_lms151_1);
        lasers.push_back(laser_lms151_2);
        lasers.push_back(laser_ldmrs);
        //lasers.push_back(swissranger);
        //lasers.push_back(stereo);
        lasers.push_back(singleCam);
        lasers.push_back(singleCamPnP);
        lasers.push_back(singleCamPnPRansac);

        vector<double> RPY;
        // ATLASCAR model rotations
        RPY.push_back(M_PI/2); // X-rotation
        RPY.push_back(0.0); // Y-rotation
        RPY.push_back(M_PI); // Z-rotation

        vector<double> translation;
        translation.push_back(-4.387/2+ 0.05); // X translation. 4.387 is the car's length
        translation.push_back(-1.702/2 + 0.05 ); // Y translation. 1.702 is the car's width
        translation.push_back(-0.46); // Z translation. 0.46 is the height of the reference LMS sensor

        visualization_msgs::Marker atlascar = addCar(RPY, translation); // builds the marker to publish
        car_pub.publish( atlascar );

        RPY.clear();
        // Clouds and lasers rotations so point cloud is aligned with Rviz grid
        RPY.push_back(0.0);
        RPY.push_back(0.0);
        RPY.push_back(55 * M_PI/180);

        visualization_msgs::MarkerArray targets_markers;
        targets_markers.markers = createTargetMarkers(clouds, lasers, RPY);
        markers_pub.publish(targets_markers);

    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::setCalibrationPoints(int numPoints)
{
    num_of_points=numPoints;
}
