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
   \file  gui_supportedsensors.cpp
   \brief Sets and gets information related to the sensors supported by the calibration application
   \author David Silva
   \date   July, 2016
 */
 
#include "calibration_gui/gui_supportedsensors.h"

#include <QDebug>

/**
   @brief Constructor for the SupportedSensors class.
 */
SupportedSensors::SupportedSensors()
{
    // Add supported sensors: (roslaunch file name, node name, is it a camera?)
    this->addSupportedSensors("Sick LMS151", "lms151", false);
    this->addSupportedSensors("Sick LD-MRS400001", "ldmrs", false);
    this->addSupportedSensors("Point Grey FL3-GE-28S4-C", "pointgrey", true);
    //this->addSupportedSensors("SwissRanger SR4000_Ethernet", "swissranger_eth", false); launch file needs to be developed
    this->addSupportedSensors("SwissRanger SR4000_USB", "swissranger_usb", false);
    this->addSupportedSensors("Microsoft Kinect 3D Sensor", "kinect_3d", false);

    for (int i = 0; i < supportedSensors.size(); i++)
        sensorCounter.push_back(0);
}

/**
   @brief Method to add a new sensor to the
   @param[in] roslaunch_name file name of the roslaunch file
   @param[in] node_name name for the sensor nodes
   @param[in] camera true if the sensor is a camera; false if not
   @return void
 */
void SupportedSensors::addSupportedSensors (QString roslaunch_name, QString node_name, bool camera)
{
    supportedSensors.push_back(roslaunch_name);
    supportedSensorsNodes.push_back(node_name);
    supportedCamera.push_back(camera);
}

/**
   @brief Method to add the required childs
   @param[in] item parent item from the QTreeWidget
   @param[in] sensorID item sensor name
   @return void
 */
void SupportedSensors::addTreeChilds(QTreeWidgetItem *item, const QString sensorID)
{
    QString ask_IP = "IP Address";

    if (sensorID == supportedSensors[0]) //Sick LMS151
        makeChild(item, ask_IP, 0);
    else if (sensorID == supportedSensors[1]) //Sick LD-MRS400001
        makeChild(item, ask_IP, 0);
    else if (sensorID == supportedSensors[2]) //Point Grey FL3-GE-28S4-C
    {
        // Nothing for now...IP is changed through FlyCap2 software
    }
    else if (sensorID == supportedSensors[3]) //SwissRanger SR4000_(Ethernet)
        makeChild(item, ask_IP, 0);
    else if (sensorID == supportedSensors[4]) // SwissRanger SR4000_(USB)
    {
        // Nothing for now
    }
    else if (sensorID == supportedSensors[5]) // Microsoft Kinect IR Sensor
    {
        // Nothing for now
    }
    else
    {
        // Alert the user because sensorID is unexpected
    }
}

/**
   @brief Creates the QTreeWidgetItem childs
   @param[in] parent is the parent QTreeWidgetItem
   @param[in] text is the text to display
   @param[in] column column of the QTreeWidgetItem child to be added
   @return void
 */
void SupportedSensors::makeChild (QTreeWidgetItem *parent, const QString text, int column)
{
    QTreeWidgetItem *item = new QTreeWidgetItem(parent);

    parent->addChild(item);

    // Source https://forum.qt.io/topic/28360/clicking-in-qtreewidget-item-while-already-editing-reopens-the-editor/5
    // Makes this item editable
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable);

    item->setText(column, text);

    item->parent()->setExpanded(true);
}

/**
   @brief Gets the roslaunch file arguments from the QTreeWidget
   @param[in] item QTreeWidgetItem
   @param[in] sensor name of the sensor
   @param[in] ballDiameter diameter of the ball inserted in the Options window
   @return roslaunch_arg a list of the roslaunch arguments
 */
QStringList SupportedSensors::roslaunchManager(QTreeWidgetItem * item, QString sensor, double ballDiameter)
{
    QString ball_diameter = "ball_diameter:=" + QString::number(ballDiameter);
    QString node_name = "node_name:=";
    QString sensorIP = "host:=";

    QTreeWidgetItem *itemchild;

    QStringList roslaunch_params = QStringList() << ball_diameter;

    for (int i = 0; i<supportedSensors.size(); i++)
    {
        if (sensor == supportedSensors[i])
        {
            sensorCounter[i] += 1;
            launchedNodes.push_back(supportedSensorsNodes[i] + "_" + QString::number(sensorCounter[i]));
            node_name += launchedNodes.last();

            if (item->childCount())
            {
                itemchild = item->child(0);
                sensorIP += itemchild->text(1);
            }
            roslaunch_params << sensorIP << node_name;
            isCamera.push_back(supportedCamera[i]);
        }
    }

    QString roslaunch_file = sensor +".launch";

    QStringList roslaunch_arg = QStringList() << "calibration_gui"
                                              << roslaunch_file
                                              << roslaunch_params;

    return roslaunch_arg;
}

/**
   @brief Gets sensor names to display in the Rviz 3D visualizer
   @param void
   @return displayNames vector of strings with the sensor names
 */
std::vector<std::string> SupportedSensors::getDisplayNames()
{
    std::vector<std::string> displayNames;
    std::vector<std::string> displayNamesRigid;
    std::vector<std::string> displayNamesPnP;
    for (int i = 0; i < launchedNodes.size(); i++)
    {
        if (isCamera[i])
        {
            displayNamesRigid.push_back(launchedNodes[i].toStdString() + "_rigid");
            displayNamesPnP.push_back(launchedNodes[i].toStdString() + "_pnp");
        }
        else
            displayNamesRigid.push_back(launchedNodes[i].toStdString());
    }

    displayNames.reserve( displayNamesRigid.size() + displayNamesPnP.size() );
    displayNames.insert( displayNames.end(), displayNamesRigid.begin(), displayNamesRigid.end() );
    displayNames.insert( displayNames.end(), displayNamesPnP.begin(), displayNamesPnP.end() );

    return displayNames;
}

/**
   @brief Clears variables
   @param void
   @return void
 */
void SupportedSensors::resetLaunchedLists()
{
    isCamera.clear();
    launchedNodes.clear();
    sensorCounter.clear();
    for (int i = 0; i < supportedSensors.size(); i++)
        sensorCounter.push_back(0);
}


