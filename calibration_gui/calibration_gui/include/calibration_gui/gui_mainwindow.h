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
   \file  gui_mainwindow.h
   \brief Header file for gui_mainwindow.cpp.
   \author David Silva
   \date   July, 2016
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTreeWidget>
#include <QStyledItemDelegate>
#include <QItemDelegate>
#include <QList>
#include <QProcess>
#include <QVector>

#include "ui_mainwindow.h"
#include "calibration_gui/gui_calibration_node.h"
#include "calibration_gui/gui_myrviz.h"
#include "calibration_gui/gui_options.h"
#include "calibration_gui/gui_supportedsensors.h"
#include "calibration_gui/gui_QProgressIndicator.h"

namespace Ui {
class MainWindow;
}

/**
   \class MainWindow
   \brief Handles the MainWindow window
   \author David Silva
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QNode *node, QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_bt_add_clicked();

    void on_bt_remove_clicked();

    void on_bt_make_reference_clicked();

    void on_actionOptions_triggered();

    void on_treeWidget_itemChanged(QTreeWidgetItem *item, int column);

    void on_treeWidget_itemSelectionChanged();

    void combobox_itemChanged(const QString &text);

    void on_bt_start_nodes_clicked();

    void on_bt_stop_nodes_clicked();

    void on_bt_calibrate_clicked();

    void on_bt_stop_calibrate_clicked();

    void NodeFinished(int exit_code, QProcess::ExitStatus exit_status);

    void calibrationFinished();

private:
    Ui::MainWindow *ui;
    QNode *qnode;
    MyViz *mRviz;
    Options *mOptions;
    QProgressIndicator* mProgress;
    SupportedSensors *mSensors;
    QVector<QProcess*> processes;
    std::vector<bool> isCamera;

    // Parameter Strings
    QString parameterBallDiameter;
    QString parameterNumPoints;
    QList<QString> supportedSensors;
    QList<QString> supportedSensorsNodes;
    QList<QString> launchedNodes;

};

/**
   \class MyItemDelegate
   \brief Delegate which allows specific columns to be editable. In this case, only column #1 is editable.

   Source: http://www.qtcentre.org/archive/index.php/t-7031.html
   \author David Silva
 */
class MyItemDelegate : public QItemDelegate
{
public:
    MyItemDelegate(QObject* parent = 0) : QItemDelegate(parent) {}

    QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
    {
        // allow only specific column to be edited, second column in this example
        if (index.column() == 1)
            return QItemDelegate::createEditor(parent, option, index);
        return 0;
    }
};

#endif // MAINWINDOW_H
