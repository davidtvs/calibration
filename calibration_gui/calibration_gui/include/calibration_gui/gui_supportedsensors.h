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
   \file  gui_supportedsensors.h
   \brief Header file for gui_supportedsensors.cpp. Sets and gets information related to the sensors supported by the calibration application
   \author David Silva
   \date   July, 2016
 */


#ifndef GUI_SUPPORTEDSENSORS_H
#define GUI_SUPPORTEDSENSORS_H

#include <QList>
#include <QTreeWidget>

/**
   \class SupportedSensors
   \brief Class to handle information related to the sensors supported by the calibration application
   \author David Silva
 */
class SupportedSensors
{
public:
    SupportedSensors();
	
    QList<QString> getSupportedSensors() { return supportedSensors; }

    QList<QString> getSupportedSensorsNodes() { return supportedSensorsNodes; }

    void addTreeChilds(QTreeWidgetItem *parent, const QString sensorID);

    QStringList roslaunchManager(QTreeWidgetItem * item, QString sensor, double ballDiameter);

    QList<QString> getLaunchedNodes() { return launchedNodes; }

    std::vector<bool> getIsCamera() { return isCamera; }

    std::vector<std::string> getDisplayNames();

    void resetLaunchedLists();

private:
    QList<QString> supportedSensors;
    std::vector<bool> supportedCamera;
    QList<QString> supportedSensorsNodes;
    QList<QString> launchedNodes;
    QList<int> sensorCounter;
    std::vector<bool> isCamera;


    void makeChild (QTreeWidgetItem *parent, const QString text, int column);
    void addSupportedSensors (QString roslaunch_name, QString node_name, bool camera);
};

#endif // GUI_SUPPORTEDSENSORS_H
