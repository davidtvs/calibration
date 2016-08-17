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
   \brief Class to set and get information related to the sensors supported by the calibration application
   \author David Silva
   \date   July, 2016
 */

#ifndef NODE_HPP_
#define NODE_HPP_


#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QMessageBox>
#include <QMutexLocker>
#include <QWaitCondition>

/**
   @class QNode
   @brief Provides a thread for ros to spin in which safely self destructs.
   See http://docs.ros.org/electric/api/qt_tutorials/html/qnode_8cpp_source.html  
 */
class QNode : public QThread {
Q_OBJECT

public:
    QNode(int argc, char** argv, const std::string &name );
    virtual ~QNode();

    bool on_init();
    bool on_init(const std::string &master_url, const std::string &host_url);
    void shutdown();
    void run();

    QStringListModel* loggingModel() { return &logging; }
    QPixmap PixmapModel (){return px;}
    const std::string& nodeName() { return node_name; }

    void setCalibrationPoints(const int numPoints) { num_of_points = numPoints; }
    void setMinDistance (const double distance) { min_distance = distance; }
    void setMaxDisplacement(const double distance) { max_displacement = distance; }
    void setNodes(const std::vector<std::string> nodes, const std::vector<bool> camera, const std::vector<std::string> names);
    void setAutoAcquisition(const bool acquisition_type) { acquisitionIsAuto = acquisition_type; }
    void setDoCalibration(const bool calibration_state) { doCalibration = calibration_state; }

signals:
    void loggingUpdated();
    void calibrationComplete();
    void showMsg( const QString& msg, QMessageBox::StandardButton* answer);

private slots:
    void msgShower(const QString& msg, QMessageBox::StandardButton* answer);

private:
    int num_of_points;
    double min_distance;
    double max_displacement;
    bool acquisitionIsAuto;
    bool doCalibration;

    std::vector<std::string> calibrationNodes;
    std::vector<bool> isCamera;
    std::vector<std::string> displayNames;

    QWaitCondition waitCondition;
    QMutex mutex;

protected:
    int init_argc;
    char** init_argv;
    QStringListModel logging;
    QPixmap px;
    const std::string node_name;
};

#endif