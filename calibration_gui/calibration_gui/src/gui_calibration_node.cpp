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
{
    connect(this, SIGNAL(showMsg( const QString&, QMessageBox::StandardButton*)),
            this, SLOT(msgShower( const QString&, QMessageBox::StandardButton*)));

}

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
    //ros::Publisher car_pub;

    ros::NodeHandle n;

    markers_pub = n.advertise<visualization_msgs::MarkerArray>( node_name + "/CalibrationPoints", 10000);
    //car_pub = n.advertise<visualization_msgs::Marker>(node_name + "/3DModel", 1);

    CircleCentroids centroids(calibrationNodes, isCamera);

    vector<pcl::PointXYZ> sensorsBallCenters;
    vector<pcl::PointXYZ> camCentroidPnP;
    vector<cv::Mat> camImage;

    // Vector for containing future pointclouds for each sensor
    vector<pcl::PointCloud<pcl::PointXYZ> > sensorClouds;
    // Vector for containing image pointclouds from cameras - solvePnP method
    vector<pcl::PointCloud<pcl::PointXYZ> > cameraCloudsPnP;
    for (int i=0; i < calibrationNodes.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ> sensorCloud;
        sensorClouds.push_back(sensorCloud);
        if (isCamera[i])
        {
            pcl::PointCloud<pcl::PointXYZ> cameraCloudPnP;
            cameraCloudsPnP.push_back(cameraCloudPnP);
        }
    }

    // Vector for containing future sensor poses
    vector<geometry_msgs::Pose> sensorPoses;
    // Vector for containing future sensor poses
    vector<geometry_msgs::Pose> cameraPosesPnP;
    for (int i=0; i < calibrationNodes.size(); i++)
    {
        geometry_msgs::Pose sensorPose;
        sensorPoses.push_back(sensorPose);
        if (isCamera[i])
        {
            geometry_msgs::Pose cameraPosePnP;
            cameraPosesPnP.push_back(cameraPosePnP);
        }
    }
    sensorPoses.front().orientation.w = 1.0; // so it can be multiplied by transformations later, only the reference sensor needs this

    // Pointclouds used for Rviz visualization
    vector<geometry_msgs::Pose> visualizationPoses;
    vector<pcl::PointCloud<pcl::PointXYZ> > visualizationClouds;


    int count=0;
    pcl::PointXYZ P[2];
    P[1].x=0;
    P[1].y=0;
    P[1].z=0;

    createDirectory( );

    qDebug() << "Calibration is going to start";

    ros::Rate loop_rate(50);

    while(count < num_of_points && ros::ok() && doCalibration)
    {
        sensorsBallCenters = centroids.getSensorsBallCenters();
        camCentroidPnP = centroids.getCamCentroidPnP();
        camImage = centroids.getCamImage();

        int finder = 0;
        bool found = false;
        while ( finder < sensorsBallCenters.size() )
        {
            if (sensorsBallCenters[finder].x == -999)
            {
                found = true;
                break;
            }
            finder++;
        }
        finder=0;
        while ( finder < camCentroidPnP.size() && !found )
        {
            if (camCentroidPnP[finder].x == -999)
            {
                found = true;
                break;
            }
            finder++;
        }

        if(!found)
        {

            P[0].x=sensorsBallCenters[0].x;
            P[0].y=sensorsBallCenters[0].y;
            P[0].z=sensorsBallCenters[0].z;

            double dist;
            dist=sqrt(pow((P[0].x-P[1].x),2) + pow((P[0].y-P[1].y),2)); // X-Y distance
            //cout<<"dist = "<<dist<<endl;

            if(dist > min_distance) // If the ball hasn't moved more than 0.15 meters the points are discarded
            {
                float diff_dist_mean=0;
                if(count>0) // code beow is skipped on the first cycle (count = 0)
                {
                    vector<float> eu_dist;
                    for (int i = 0; i < sensorsBallCenters.size(); i++)
                    {
                        // Calculation of the distance between the corrent center of the ball detected and the previous
                        eu_dist.push_back( pointEuclideanDistance (sensorClouds[i].points[count-1], sensorsBallCenters[i]) );
                        // Sums the squared difference between the reference sensor (first sensor on sensors_ball_centers) and all the other sensors
                        diff_dist_mean += abs(eu_dist.front() - eu_dist.back());
                        cout << endl << "points = " << i << " " << sensorsBallCenters[i] << " " << diff_dist_mean  << endl;
                        for (int i=0; i < eu_dist.size(); ++i)
                          std::cout << eu_dist[i] << ' ';

                        std::cout << std::endl;
                    }
                    // Mean squared distance - high diff_dist_mean means that the ball displacement was not equal for every sensor, which means something is wrong
                    diff_dist_mean = diff_dist_mean/sensorsBallCenters.size();
                    qDebug() << sensorsBallCenters.size();
                }
                cout << "diff_dist_mean = " << diff_dist_mean << endl;

                if(diff_dist_mean <= max_displacement)    // the limit is set by the max_displacement variable in meters. If it's higher these points will be discarded
                {
                    int cameraCounter = 0;
                    for ( int i = 0; i < sensorsBallCenters.size(); i++ )
                    {
                        cout << "crash" << sensorsBallCenters[i] << endl;
                        sensorClouds[i].push_back(sensorsBallCenters[i]); // sensorCLouds now contains ball center points for every sensor
                        qDebug() << "nocrash";
                        if (isCamera[i])
                        {
                            cameraCloudsPnP[cameraCounter].push_back(camCentroidPnP[cameraCounter]);
                            string imgPath = file_path + "img_" + calibrationNodes[i] +"_" + boost::lexical_cast<std::string>(count) + ".jpg";
                            cout << imgPath << "\n" << camImage.size() << endl;
                            imwrite( imgPath, camImage[cameraCounter] );

                            cameraCounter++;
                            qDebug() << cameraCounter;
                        }
                    }


                    // Saving all point clouds to a PCD file
                    cameraCounter = 0;
                    for ( int i = 0; i < sensorClouds.size(); i++ )
                    {
                        pcl::io::savePCDFileASCII(file_path + calibrationNodes[i] + ".pcd", sensorClouds[i]);
                        if (isCamera[i])
                        {
                            pcl::io::savePCDFileASCII(file_path + calibrationNodes[i] + "_PnP.pcd", cameraCloudsPnP[cameraCounter]);
                            cameraCounter++;
                        }
                    }

                    qDebug() << "pcd save";

                    // PointClouds and Poses visualization for Rviz (vector concatenation can probably be improved)
                    visualizationPoses.clear();
                    visualizationClouds.clear();
                    visualizationPoses.reserve( sensorPoses.size() + cameraPosesPnP.size() );
                    visualizationPoses.reserve( sensorClouds.size() + cameraCloudsPnP.size() );

                    visualizationPoses.insert( visualizationPoses.end(), sensorPoses.begin(), sensorPoses.end() );
                    visualizationPoses.insert( visualizationPoses.end(), cameraPosesPnP.begin(), cameraPosesPnP.end() );
                    visualizationClouds.insert( visualizationClouds.end(), sensorClouds.begin(), sensorClouds.end() );
                    visualizationClouds.insert( visualizationClouds.end(), cameraCloudsPnP.begin(), cameraCloudsPnP.end() );

                    visualization_msgs::MarkerArray targets_markers;
                    targets_markers.markers = createTargetMarkers(visualizationClouds, visualizationPoses);
                    markers_pub.publish(targets_markers);


                    P[1].x=sensorsBallCenters[0].x;
                    P[1].y=sensorsBallCenters[0].y;
                    P[1].z=sensorsBallCenters[0].z;
                    //cout<<"count "<<count+1<<endl;

                    count++;
                    if (count < num_of_points && !acquisitionIsAuto) // If acquisitionIsAuto is false then the user is prompted before acquiring points.
                    {
                        QMessageBox::StandardButton reply;

                        QMutexLocker locker(&mutex);
                        QString msg = "Point " + QString::number(count) + " of " + QString::number(num_of_points)
                                + " done.\nPress OK to acquire the next point.";
                        emit showMsg(msg, &reply);
                        waitCondition.wait(&mutex);
                    }
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (doCalibration)
    {

        int cameraCounter = 0;
        for (int i = 1; i < sensorsBallCenters.size(); i++) // starts with i=1 because for i=0 the target and uncalibrated sensor are the same
        {
            // Estimating rigid transform between target sensor and other sensors
            estimateTransformation(sensorPoses[i], sensorClouds.front(), sensorClouds[i],
                                   calibrationNodes.front(), calibrationNodes[i]);
            if (isCamera[i])
            {
                estimateTransformationCamera(cameraPosesPnP[cameraCounter], sensorClouds.front(), cameraCloudsPnP[cameraCounter],
                                             calibrationNodes.front(), calibrationNodes[i], camImage[cameraCounter], true, true);
                cameraCounter++;
            }
        }

        // Implement 3D models with translation and rotation sliders
        /*vector<double> RPY;
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
        RPY.push_back(55 * M_PI/180);*/

        // PointClouds and Poses visualization for Rviz (vector concatenation can probably be improved)
        visualizationPoses.clear();
        visualizationClouds.clear();
        visualizationPoses.reserve( sensorPoses.size() + cameraPosesPnP.size() );
        visualizationPoses.reserve( sensorClouds.size() + cameraCloudsPnP.size() );

        visualizationPoses.insert( visualizationPoses.end(), sensorPoses.begin(), sensorPoses.end() );
        visualizationPoses.insert( visualizationPoses.end(), cameraPosesPnP.begin(), cameraPosesPnP.end() );
        visualizationClouds.insert( visualizationClouds.end(), sensorClouds.begin(), sensorClouds.end() );
        visualizationClouds.insert( visualizationClouds.end(), cameraCloudsPnP.begin(), cameraCloudsPnP.end() );

        visualization_msgs::MarkerArray targets_markers;
        targets_markers.markers = createTargetMarkers(visualizationClouds, visualizationPoses);
        markers_pub.publish(targets_markers);

        cout<<"Calibration complete!"<<endl;
    }

    emit calibrationComplete(); // used to signal the gui that the calibration is complete
}


void QNode::setLaunchedNodes(const vector<string> nodes, const vector<bool> camera)
{
    qDebug() << "setLaunchedNodes";
    calibrationNodes = nodes;
    isCamera = camera;
}


void QNode::msgShower(const QString& msg, QMessageBox::StandardButton* answer)
{
    QMutexLocker locker(&mutex);
    *answer = QMessageBox::information(0, "Next Point", msg);
    waitCondition.wakeOne();
}
