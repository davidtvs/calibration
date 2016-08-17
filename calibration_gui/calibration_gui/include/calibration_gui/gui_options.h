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
   \file  gui_options.h
   \brief Header file for gui_options.cpp. Options window related to the calibration process.
   \author David Silva
   \date   July, 2016
 */

#ifndef OPTIONS_H
#define OPTIONS_H

#include <QDialog>
#include <QCloseEvent>

namespace Ui {
class Options;
}

/**
   \class Options
   \brief Class to handle the Options window
   \author David Silva
 */
class Options : public QDialog
{
    Q_OBJECT

public:
    explicit Options(QWidget *parent = 0);
    ~Options();

    QString getBallDiameter() { return ball_diameter; }

    QString getNumCalibPoints(){ return num_calibration_points; }

    QString getMinDistance(){ return min_distance; }

    QString getMaxDisplacement(){ return max_displacement; }

    bool getAutoAcquisition(){ return auto_acquisition; }

private slots:
    void on_buttonBox_accepted();

    void closeEvent(QCloseEvent *event);

private:
    Ui::Options *ui;

    QString ball_diameter;
    QString num_calibration_points;
    QString min_distance;
    QString max_displacement;
    bool auto_acquisition;
};

#endif // OPTIONS_H
