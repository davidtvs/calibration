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

    // Vector for containing future pointclouds for each sensor
    vector<pcl::PointCloud<pcl::PointXYZ> > sensorClouds;
    for (int i=0; i < calibrationNodes.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ> sensorCloud;
        sensorClouds.push_back(sensorCloud);
    }

    // Vector for containing future sensor poses
    vector<geometry_msgs::Pose> sensorPoses;
    for (int i=0; i < calibrationNodes.size(); i++)
    {
        geometry_msgs::Pose sensorPose;
        sensorPoses.push_back(sensorPose);
    }
    sensorPoses[0].orientation.w = 1.0; // so it can be multiplied by transformations later


    ros::NodeHandle n;
    // Get number of points to collect from TreeWidget
    // Should be on cell (1,1), but just in case this is changed the code below searches for the string in the whole tree
    //int num_of_points = MainWindow::treeWidgetFindText( "Number of Calibration Points" );
    qDebug() << num_of_points;


    markers_pub = n.advertise<visualization_msgs::MarkerArray>( node_name + "/CalibrationPoints", 10000);
    car_pub = n.advertise<visualization_msgs::Marker>(node_name + "/3DModel", 1);

    CircleCentroids centroids(calibrationNodes, isCamera);
    int count=0;
    pcl::PointXYZ P[2];
    P[1].x=0;
    P[1].y=0;
    P[1].z=0;

    sensorPoses.front().orientation.w=1.0; // so it can be multiplied by transformations later

    vector<cv::Point3f> objectPoints;
    vector<cv::Point2f> imagePoints;

    createDirectory( );

    qDebug() << "Calibration is going to start";

    ros::Rate loop_rate(50);

    while(count < num_of_points && ros::ok())
    {
        int finder = 0;
        bool found = false;
        while ( finder < centroids.sensors_ball_centers.size()-1 )
        {
            if (centroids.sensors_ball_centers[finder].x == -999)
            {
                found = true;
                break;
            }
            finder++;
        }
        // Change condition below when subscribing is generic
        if(!found)
        {

            P[0].x=centroids.sensors_ball_centers[0].x;
            P[0].y=centroids.sensors_ball_centers[0].y;
            P[0].z=centroids.sensors_ball_centers[0].z;

            double dist;
            dist=sqrt(pow((P[0].x-P[1].x),2) + pow((P[0].y-P[1].y),2)); // X-Y distance
            cout<<"dist = "<<dist<<endl;

            if(dist>0.15) // If the ball hasn't moved more than 0.15 meters the points are discarded          // MAKE THIS A VARIABLE
            {
                float diff_dist_mean=0;
                if(count>0) // code beow is skipped on the first cycle (count = 0)
                {
                    vector<float> eu_dist;
                    for (int i = 0; i < centroids.sensors_ball_centers.size(); i++)
                    {
                        // Calculation of the distance between the corrent center of the ball detected and the previous
                        eu_dist.push_back( pointEuclideanDistance (sensorClouds[i].points[count-1], centroids.sensors_ball_centers[i]) );
                        // Sums the squared difference between the reference sensor (first sensor on sensors_ball_centers) and all the other sensors
                        diff_dist_mean += pow(eu_dist.front() - eu_dist.back(), 2);
                    }
                    // Mean squared distance - high diff_dist_mean means that the ball displacement was not equal for every sensor, which means something is wrong
                    diff_dist_mean = diff_dist_mean/centroids.sensors_ball_centers.size();
                }
                std::cout << "diff_dist_mean = " << diff_dist_mean << std::endl;
                if(diff_dist_mean<0.15)    // the limit is 0.15 meters. If it's higher these points will be discarded
                {

                    for ( int i = 0; i < centroids.sensors_ball_centers.size(); i++ )
                        sensorClouds[i].push_back(centroids.sensors_ball_centers[i]); // sensorCLouds now contains ball center points for every sensor

                    /* FUTURE WORK - implement a way to know which type of sensor is being used (camera, laser) */
                    for (int i = 0; i < centroids.sensors_ball_centers.size(); i++)
                    {
                        if (isCamera[i])
                        {
                            /*string filename_obj_img = calibrationNodes[i] + "_obj_img_points.txt"; // file filename that stores object points and image points
                            string tmp_str = file_path + filename_obj_img;

                            const char *FilePath_obj_img = tmp_str.c_str();

                            FILE* pFile = fopen(FilePath_obj_img, "w");
                            fprintf(pFile, "ObjectPoints (m)\tImagePoints (pixel)\n");
                            fprintf(pFile, "x;y;z\tx;y;r\n");

                            pFile = fopen(FilePath_obj_img, "a");
                            fprintf(pFile, "%F;%F;%F\t", centroids.sensors_ball_centers[0].x, centroids.sensors_ball_centers[0].y, centroids.sensors_ball_centers[0].z);

                            // save imagePoints to file
                            fprintf(pFile, "%F;%F;%F\n", centroids.sensors_ball_centers[i].x, centroids.sensors_ball_centers[i].y, centroids.sensors_ball_centers[i].z);
                            fclose(pFile);

                            string imgPath = file_path + "img_" + calibrationNodes[i] +"_" + boost::lexical_cast<std::string>(count) + ".jpg";
                            cv::Mat img;
                            img = cv_bridge::toCvShare(centroids.camImage, "bgr8")->image;
                            imwrite( imgPath, img );*/
                        }
                    }


                    // Saving all point clouds to a PCD file
                    for ( int i = 0; i < sensorClouds.size(); i++ )
                        pcl::io::savePCDFileASCII(file_path + calibrationNodes[i] + ".pcd", sensorClouds[i]);



                    visualization_msgs::MarkerArray targets_markers;
                    targets_markers.markers = createTargetMarkers(sensorClouds, sensorPoses);
                    markers_pub.publish(targets_markers);


                    P[1].x=centroids.sensors_ball_centers[0].x;
                    P[1].y=centroids.sensors_ball_centers[0].y;
                    P[1].z=centroids.sensors_ball_centers[0].z;
                    cout<<"count "<<count+1<<endl;

                    cout<<"Press to select another point";
                    cin.ignore();
                    count++;
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


    cout<<"Press Enter to Continue";
    cin.ignore();

    /*// What happens below????
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
    markers_pub.publish(targets_markerss);*/

    // Estimating rigid transform between target sensor and other sensors
    for (int i = 1; i < centroids.sensors_ball_centers.size(); i++) // starts with i=1 because for i=0 the target and uncalibrated sensor are the same
    {
        estimateTransformation(sensorPoses[i], sensorClouds.front(), sensorClouds[i],
                               calibrationNodes.front(), calibrationNodes[i]);
    }

    // IMPLEMENT THIS
    /*estimateTransformationCamera(singleCamPnP, objectPoints, imagePoints, "lms1_camera_calib", true, false); // Transformation estimation with solvePnP
    estimateTransformationCamera(singleCamPnPRansac, objectPoints, imagePoints, "lms1_camera_calib", true, true); // Transformation estimation with solvePnPRansac
    */


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
    targets_markers.markers = createTargetMarkers(sensorClouds, sensorPoses, RPY);
    markers_pub.publish(targets_markers);

    cout<<"Calibration complete!"<<endl;

    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::setCalibrationPoints(int numPoints)
{
    qDebug() << "setCalibrationPoints";
    num_of_points=numPoints;
}

void QNode::setLaunchedNodes(vector<string> nodes, vector<bool> camera)
{
    qDebug() << "setLaunchedNodes";
    calibrationNodes=nodes;
    isCamera=camera;
}
