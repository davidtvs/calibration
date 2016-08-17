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
   \file  gui_options.cpp
   \brief Options window related to the calibration process
   \author David Silva
   \date   July, 2016
 */

#include "calibration_gui/gui_options.h"
#include "ui_options.h"

/**
   @brief Constructor for the Options class.
   @param parent is the parent widget
 */
Options::Options(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Options)
{
    ui->setupUi(this);

    ui->ledit_ball_diameter->setValidator( new QDoubleValidator(0, 100, 3, this) );
    ui->ledit_min_distance->setValidator( new QDoubleValidator(0, 100, 3, this) );
    ui->ledit_max_disp->setValidator( new QDoubleValidator(0, 100, 3, this) );
    ui->ledit_num_calib_points->setValidator( new QIntValidator(0, 100, this) );

    ui->ledit_ball_diameter->setText("0.967");
    ui->ledit_min_distance->setText("0.500");
    ui->ledit_max_disp->setText("0.100");
    ui->ledit_num_calib_points->setText("20");

    ball_diameter = ui->ledit_ball_diameter->text();
    num_calibration_points = ui->ledit_num_calib_points->text();
    min_distance = ui->ledit_min_distance->text();
    max_displacement = ui->ledit_max_disp->text();
    auto_acquisition = ui->rb_auto->isChecked();
}

/**
   @brief GUI Main Window destructor
 */
Options::~Options()
{
    delete ui;
}

/**
   @brief OK button SLOT.
   @param void
   @return void
 */
void Options::on_buttonBox_accepted()
{
    ball_diameter = ui->ledit_ball_diameter->text();
    num_calibration_points = ui->ledit_num_calib_points->text();
    min_distance = ui->ledit_min_distance->text();
    max_displacement = ui->ledit_max_disp->text();
    auto_acquisition = ui->rb_auto->isChecked();
}

/**
   @brief Close event SLOT.
   @param event 
   @return void
 */
void Options::closeEvent(QCloseEvent *event)
{
    ui->ledit_ball_diameter->setText(ball_diameter);
    ui->ledit_min_distance->setText(min_distance);
    ui->ledit_max_disp->setText(max_displacement);
    ui->ledit_num_calib_points->setText(num_calibration_points);
    if(auto_acquisition)
    {
        ui->rb_auto->setChecked(true);
    }
    event->accept();
}
