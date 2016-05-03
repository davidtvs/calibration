/*****************************************************************************
** Includes
*****************************************************************************/

// ROS includes
#include <ros/ros.h>
#include <ros/network.h>

// Project includes
#include "calibration_gui/gui_calibration_node.hpp"
#include "calibration_gui/gui_mainwindow.h"
#include "ui_mainwindow.h"

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
    start();
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
    start();
    return true;
}

void QNode::run() {
    ros::Rate loop_rate(50);
    while ( ros::ok() ) {
        ros::NodeHandle n;
        // Get number of points to collect from TreeWidget
        // Should be on cell (1,1), but just in case this is changed the code below searches for the string in the whole tree
        //int num_of_points = MainWindow::treeWidgetFindText( "Number of Calibration Points" );
        qDebug() << num_of_points;

        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::setCalibrationPoints(int numPoints)
{
    num_of_points=numPoints;
}
