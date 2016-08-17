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
   \file  gui_main.cpp
   \brief Starts the calibration node and its GUI
   \author David Silva
   \date   July, 2016
 */

#include "calibration_gui/gui_mainwindow.h"
#include "calibration_gui/gui_myrviz.h"

#include <QApplication>
#include <ros/ros.h>

/**
   @brief Main function of the GUI for the extrinsic calibration package
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
    /*if( !ros::isInitialized() )
      {
        ros::init( argc, argv, "myviz", ros::init_options::AnonymousName );
      }*/
    QApplication a(argc, argv);
    QNode node(argc,argv, "calibration_gui");
    MainWindow w(&node);
    w.show();

    a.exec();

    return 0;
}

// ====================================================================================
// Useful sources:
// http://wiki.ros.org/qt_create/Tutorials/Qt%20App%20Templates?action=show&redirect=qt_ros%252FTutorials%252FQt+App+Templates+-+Electric
// https://github.com/decoderdan/projectPhoenix/wiki/Creating-a-ROS-qt-gui-node
// https://github.com/dxydas/ros-bioloid
// https://github.com/dazhbog/tanky_gui
