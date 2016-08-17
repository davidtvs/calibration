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
   \file  gui_myrviz.cpp
   \brief Builds a 3D visualizer based on Rviz to display information acquired by the sensors and calibration results.
   \author David Silva
   \date   July, 2016
 */

#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QDebug>
#include <QVariant>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "calibration_gui/gui_myrviz.h"

/**
   @brief Constructor for the Rviz 3D visualizer widget
   See http://docs.ros.org/kinetic/api/librviz_tutorial/html/index.html
   @param[in] parent is the parent widget
 */
MyViz::MyViz( QWidget* parent )
  : QWidget( parent )
{

  // Construct render panel.
  render_panel_ = new rviz::RenderPanel();
  // Layout
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );

  manager_->setFixedFrame( "/my_frame3" );

  manager_->initialize();
  manager_->startUpdate();

  // Create a Grid display.
  rviz::Display* grid_;
  grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );

  ROS_ASSERT( grid_ != NULL );
}

/**
   @brief GUI Main Window destructor
 */
MyViz::~MyViz()
{
  delete manager_;
}

/**
   @brief Method to subscribe to a topic
   @param[in] qnode_name topic to subscribe name
   @return void
 */
void MyViz::subscribeTopics(const QString qnode_name)
{
    QString calibPointsTopic = "/" + qnode_name + "/CalibrationPoints";
    qDebug() << "Calibration points topic:" + calibPointsTopic;
   // QString Model3DTopic = "/" + qnode_name + "/3DModel";
    //qDebug() << "Calibration points topic:" + Model3DTopic;


    rviz::Display* markerarray;

    markerarray= manager_->createDisplay( "rviz/MarkerArray", "MarkerArray", true );
    markerarray->subProp( "Marker Topic" )->setValue(calibPointsTopic);

    ROS_ASSERT( markerarray != NULL );

    /*rviz::Display* marker;
    marker= manager_->createDisplay( "rviz/Marker", "Marker", true );
    marker->subProp( "Marker Topic" )->setValue( Model3DTopic );

    ROS_ASSERT( marker != NULL );*/
}
