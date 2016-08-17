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
   \file  gui_mainwindow.cpp
   \brief Extrinsic calibration package Graphical User Interface Main Window
   \author David Silva
   \date   July, 2016
 */

#include "calibration_gui/gui_mainwindow.h"

#include <QDebug>
#include <QLineEdit>
#include <QToolButton>
#include <QAbstractButton> // ToolButton setText
#include <QComboBox>
#include <QCheckBox>
#include <QHeaderView>
#include <QFileDialog>
#include <QMdiArea>
#include <QTreeView>
#include <QMessageBox>

/**
   @brief GUI Main Window constructor. Sets up the GUI, connects signals to slots and initialization of variables.
   @param[in] node pointer to calibration ROS node
   @param[in] parent is the parent widget
 */
MainWindow::MainWindow(QNode *node, QWidget *parent)
    :QMainWindow(parent)
    ,ui(new Ui::MainWindow)
    ,qnode(node)
{
    ui->setupUi(this);

    // Start Calibration node
    qnode->on_init();

    connect(qnode, SIGNAL(calibrationComplete()), this, SLOT(calibrationFinished()));

    // Initialise classes
    mRviz = new MyViz();
    mOptions = new Options();
    mSensors = new SupportedSensors();
    mProgress = new QProgressIndicator();

    mProgress->hide();
    ui->horizontalLayout->insertWidget(0, mProgress);

    supportedSensors = mSensors->getSupportedSensors();
    supportedSensorsNodes = mSensors->getSupportedSensorsNodes();

    // Add rviz to mdiArea as a subwindow and maximize it
    ui->mdiArea_calibration->addSubWindow(mRviz, Qt::FramelessWindowHint); // FramelessWindowHint removes close, minimize and maximize title bar
    mRviz->showMaximized();

    // Header bar text
    ui->treeWidget->headerItem()->setText(0, "Calibration Manager");
    ui->treeWidget->headerItem()->setText(1, "");
    // Item editor delegate, so we can control what the user can edit in the treeWidget
    ui->treeWidget->setItemDelegate(new MyItemDelegate(ui->treeWidget));

    // Add first sensor (reference sensor) to the TreeWidget
    // Add ComboBox with avaiable lasers and cameras
    QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);
    ui->treeWidget->addTopLevelItem(item);
    QComboBox *deviceComboBox = new QComboBox();
    deviceComboBox->addItems(supportedSensors);
    ui->treeWidget->setItemWidget(item,0,deviceComboBox);

    // Add text indicating this is the reference sensor
    item->setText(1, "Reference Sensor");
    ui->treeWidget->setCurrentItem(item); // sets selection

    // Add childs to the item
    mSensors->addTreeChilds(item, supportedSensors[0]);


    // Resize Columns to fit text
    ui->treeWidget->resizeColumnToContents(0);

    // Set label text to empty
    ui->label_kill_nodes->setText("");

    // Disable buttons
    ui->bt_stop_nodes->setEnabled(false);
    ui->bt_calibrate->setEnabled(false);
    ui->bt_stop_calibrate->setEnabled(false);

    // Connecting signals to slots
    connect(ui->bt_options, SIGNAL(clicked()), this,
            SLOT(on_actionOptions_triggered())); // Options button
    connect(deviceComboBox, SIGNAL(currentIndexChanged(QString)), this,
            SLOT(combobox_itemChanged(QString))); // Combobox
}

/**
   @brief GUI Main Window destructor
 */
MainWindow::~MainWindow()
{
    delete mRviz;
    delete ui;
}


/**
   @brief Add button SLOT. Adds an item to the QTreeWidget.
   @param void
   @return void
 */
void MainWindow::on_bt_add_clicked()
{
    // Add root to TreeWidget

    // ComboBox with avaiable sensors
    QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);

    ui->treeWidget->addTopLevelItem(item);

    QComboBox *deviceComboBox = new QComboBox();
    deviceComboBox->addItems(supportedSensors);

    ui->treeWidget->setItemWidget(item,0,deviceComboBox);

    // CheckBox so the user can chose to calibrate the device or not
    QCheckBox *deviceCheckBox = new QCheckBox();
    deviceCheckBox->setCheckState(Qt::Checked);
    ui->treeWidget->setItemWidget(item,1,deviceCheckBox);

    // Connect newly created combobox signal to a slot
    connect(deviceComboBox, SIGNAL(currentIndexChanged(QString)), this,
            SLOT(combobox_itemChanged(QString)));


    // Add root's childs

    /* ComoboBox by default selects the first sensor on it's list (index 0),
     * so we add the childs of the first item on the supportedSensors list
     */
    mSensors->addTreeChilds(item, supportedSensors[0]);

}

/**
   @brief Remove button SLOT. Removes the selected item from the QTreeWidget.
   Only avalable when parent items are selected.
   @param void
   @return void
 */
void MainWindow::on_bt_remove_clicked()
{
    QList<QTreeWidgetItem*> items = ui->treeWidget->selectedItems();
    QTreeWidgetItem *pp = NULL;


    if ( !items.isEmpty() )
    {
        foreach (QTreeWidgetItem *item, items)
        {
            qDebug() << "start delete";
            delete item;
            qDebug() << "item deleted";
        }
    }
}

/**
   @brief Reference button SLOT. Makes the selected sensor item the reference sensor.
   Only avalable when parent items are selected.
   @param void
   @return void
 */
void MainWindow::on_bt_make_reference_clicked()
{
    qDebug() << "on_bt_make_reference_clicked";

    /* Since QTreeWidget doesn't directly support moving QTreeWidgetItems this Slot uses a
     * workaround to accomplish it.
     * The item will be deleted and then inserted at the new position.
     */

    QTreeWidgetItem* item = ui->treeWidget->currentItem(); // Gets currently selected item
    int row  = ui->treeWidget->currentIndex().row(); // Gets the row of the item

    QTreeWidgetItem* old_item = ui->treeWidget->topLevelItem(0); // Pointer to the old reference item

    if (item && row > 0 && old_item)
    {
        QCheckBox *deviceCheckBox = new QCheckBox;
        deviceCheckBox->setCheckState(Qt::Checked);

        old_item->setText(1, "");
        ui->treeWidget->setItemWidget(old_item, 1, deviceCheckBox);

        /* Since the item has to be deleted in order to be moved the combobox selection is going to be copied
         if this is not done, the selection is lost.*/
        QWidget *widget = ui->treeWidget->itemWidget(item, 0); // pointer to combobox widget
        QString sensor = qobject_cast<QComboBox*>(widget)->currentText(); // Gets ComboBox current text, which is a sensor
        QComboBox *deviceComboBox = new QComboBox(); // create a new combobox
        deviceComboBox->addItems(supportedSensors); // add supported sensors

        deviceComboBox->setCurrentIndex(deviceComboBox->findText(sensor)); // find text and select it

        // Now the item is removed, inserted and the new combobox equal to the one lost, is inserted
        ui->treeWidget->takeTopLevelItem(row); // Removes the item
        ui->treeWidget->insertTopLevelItem(0, item); // Inserts item at the top of the treewidget (0)
        ui->treeWidget->setItemWidget(item, 0, deviceComboBox); // adds combobox to the item
        item->setText(1, "Reference Sensor");
        ui->treeWidget->setCurrentItem(item); // sets selection
        item->setExpanded(true);

        // Connecting new Combobox to slot
        connect(deviceComboBox, SIGNAL(currentIndexChanged(QString)), this,
                SLOT(combobox_itemChanged(QString))); //
    }
}

/**
   @brief Options button SLOT. Launches the Options window.
   @param void
   @return void
 */
void MainWindow::on_actionOptions_triggered()
{
    mOptions->setModal(true);
    mOptions->exec();
}

/**
   @brief SLOT for changed items.
   For Debugging purposes only.
   @param[in] item QTreeWidgetItem changed by the user
   @param[in] column column that contains \p item
   @return void
 */
void MainWindow::on_treeWidget_itemChanged(QTreeWidgetItem *item, int column)
{
    qDebug() << "on_treeWidget_itemChanged";
    QString changedText = item->text(column); // column should always be 1, but this is more generic
    QString parameter = item->text(0); // get changed paremeter name
    qDebug() << changedText << " " << column << " " << parameter;
}

/**
   @brief Enables/disables buttons according to the currently selected item in the QTreeWidget
   @param void
   @return void
 */
void MainWindow::on_treeWidget_itemSelectionChanged()
{
    QList<QTreeWidgetItem *> selectedItems = ui->treeWidget->selectedItems();

    if (!selectedItems.isEmpty()) // not empty
    {
        foreach (QTreeWidgetItem *item, selectedItems)
        {
            if (item->parent()) // has parent
            {
                ui->bt_remove->setEnabled(false);
                ui->bt_make_reference->setEnabled(false);
            }
            else if (item->text(0) == "Calibration Options" || item->text(1) == "Reference Sensor")
            {
                ui->bt_remove->setEnabled(false);
                ui->bt_make_reference->setEnabled(false);
            }
            else
            {
                ui->bt_remove->setEnabled(true);
                ui->bt_make_reference->setEnabled(true);
            }
        }
    }
}

/**
   @brief SLOT called after the user selects and item fom the QComboBox
   @param[in] text QComboBox item text
   @return void
 */
void MainWindow::combobox_itemChanged(const QString &text)
{
    // Source: http://stackoverflow.com/questions/26212722/how-to-get-index-of-rows-on-click-event-of-qpushbutton-in-qtreewidget
    for(int i = 0 ; i < ui->treeWidget->topLevelItemCount() ; i++)
    {
        QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);
        if (ui->treeWidget->itemWidget(item, 0) == sender())
        {
            qDebug() << i << " " << text;
            item->takeChildren();
            mSensors->addTreeChilds(item, text);
        }
    }
}

/**
   @brief Start Nodes button SLOT. Starts the relevant nodes for each of the added sensors.
   Nodes are launched according to their respective roslaunch files.
   @param void
   @return void
 */
void MainWindow::on_bt_start_nodes_clicked()
{
    qDebug() << "in start nodes";

    double ballDiameter = mOptions->getBallDiameter().toDouble();

    QString msg = "Ball diameter is set to " + QString::number(ballDiameter) + " meters.\nDo you wish to continue?";
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Start Nodes", msg,
                                  QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes)
    {
        ui->bt_start_nodes->setEnabled(false);

        processes.clear();

        QString program = "roslaunch";
        QStringList arguments;

        // Go trough the QTreewidget and launch nodes according to the user-selected sensors
        for (int i=0; i < ui->treeWidget->topLevelItemCount(); i++)
        {

            QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i); // Get item containing combobox
            QWidget *widget = ui->treeWidget->itemWidget(item, 1);

            // For i=1 there is no checkbox because the reference sensor will always be launched
            if (i==0 || qobject_cast<QCheckBox*>(widget)->checkState() == Qt::Checked) // Do not reverse the order, for i=1, widget is NULL
            {
                widget = ui->treeWidget->itemWidget(item, 0); // ComboBox widget is in column 0
                QString sensor = qobject_cast<QComboBox*>(widget)->currentText(); // Gets ComboBox current text, which is a sensor

                arguments = mSensors->roslaunchManager(item, sensor, ballDiameter);

                qDebug() << "Launching with:" << program << arguments.join(" ");

                processes.push_back(new QProcess(this));
                connect(processes.last(), SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(NodeFinished(int, QProcess::ExitStatus)));

                processes.last()->start(program, arguments);
            }
        }

        launchedNodes = mSensors->getLaunchedNodes();
        isCamera = mSensors->getIsCamera();

        ui->bt_stop_nodes->setEnabled(true);
        ui->bt_calibrate->setEnabled(true);
    }
}

/**
   @brief Stop Nodes button SLOT. Terminates the previously launched nodes 
   @param void
   @return void
 */
void MainWindow::on_bt_stop_nodes_clicked()
{
    qDebug() << "Nodes to be killed" << launchedNodes;

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Stop Nodes", "Are you sure you want to stop all launched nodes?",
                                  QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes)
    {

        ui->bt_stop_nodes->setEnabled(false);
        ui->bt_calibrate->setEnabled(false);

        mProgress->startAnimation();
        mProgress->show();

        ui->label_kill_nodes->setText("Terminating nodes...Please wait.");

        foreach (QProcess *process, processes)
        {
            qDebug() << "Sending terminate order to nodes:" << launchedNodes.first();
            process->terminate();
            /*QString nodeName = "/" + launchedNodes.first() + "/" + launchedNodes.first();
            QProcess nodeKiller;
            nodeKiller.start("rosnode", QStringList() << "kill" << nodeName);
            if (nodeKiller.waitForStarted(-1))
            {
                while(nodeKiller.waitForReadyRead(-1))
                    qDebug() <<  nodeKiller.readAllStandardOutput();
            }*/
            launchedNodes.removeFirst();
            qDebug() << "Nodes remaining:" << launchedNodes;
        }
        qDebug() << "Sent kill orders to all nodes" << launchedNodes;
    }
}

/**
   @brief Calibrate button SLOT. Starts the calibration node.
   See gui_calibration_node.cpp
   @param void
   @return void
 */
void MainWindow::on_bt_calibrate_clicked()
{
    int num_calib_points = mOptions->getNumCalibPoints().toInt();
    double min_distance = mOptions->getMinDistance().toDouble();
    double max_displacement = mOptions->getMaxDisplacement().toDouble();
    bool auto_acquisition = mOptions->getAutoAcquisition();

    QString str_auto_acquisition = "User prompt";
    if (auto_acquisition)
        str_auto_acquisition = "Automatic";

    QString msg = "Number of calibration points set to " + QString::number(num_calib_points)
            + ".\nMinimum distance between calibration points set to " + QString::number(min_distance)
            + " meters.\nMaximum ball center displacement error set to " + QString::number(max_displacement)
            + " meters.\nAquisition type set to \"" + str_auto_acquisition
            + "\" \n\nDo you wish to continue?";

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Start Calibration", msg,
                                  QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes)
    {
        ui->bt_calibrate->setEnabled(false);

        std::vector<std::string> vec;

        foreach( QString str, launchedNodes) {
            vec.push_back(str.toStdString());
        }

        qnode->setNodes(vec, isCamera, mSensors->getDisplayNames());
        qnode->setCalibrationPoints(num_calib_points);
        qnode->setMinDistance(min_distance);
        qnode->setMaxDisplacement(max_displacement);
        qnode->setAutoAcquisition(auto_acquisition);
        qnode->setDoCalibration(true);

        // start() is a QThread function. It calls QNode::run automatically
        qnode->start();

        ui->bt_stop_calibrate->setEnabled(true);

        QString qnode_name = QString::fromStdString(qnode->nodeName());
        mRviz->subscribeTopics(qnode_name);
    }
}

/**
   @brief Stop Calibration button SLOT. Stops the current calibration process.
   See gui_calibration_node.cpp
   @param void
   @return void
 */
void MainWindow::on_bt_stop_calibrate_clicked()
{
    qnode->setDoCalibration(false);
}

/**
   @brief SLOT for the signal sent when a previously launched sensor node dies/is killed.
   @param[in] exit_code is the exit code of the process
   @param[in] exit_status is the  exit status of the process
   @return void
 */
void MainWindow::NodeFinished(int exit_code, QProcess::ExitStatus exit_status)
{
    for (int i = 0; i < processes.size(); i++)
    {
        if (processes[i]->state() == QProcess::NotRunning)
        {
            processes[i]->deleteLater();
            processes.remove(i);
            launchedNodes.removeAt(i);
            if (!exit_code)
                qDebug() << "Process" << i << "finished normally.";
            else
                qDebug() << "Process" << i << "crashed.";
        }
    }

    if (processes.size() == 0)
    {
        launchedNodes.clear();
        isCamera.clear();
        mSensors->resetLaunchedLists();

        ui->bt_stop_nodes->setEnabled(false);
        ui->bt_calibrate->setEnabled(false);
        ui->bt_stop_calibrate->setEnabled(false);
        ui->bt_start_nodes->setEnabled(true);

        if (mProgress->isAnimated())
        {
            //ui->horizontalLayout->removeWidget(mProgress);
            mProgress->stopAnimation();
            mProgress->hide();
            ui->label_kill_nodes->setText("");
        }
    }
    qDebug() << processes;
}

/**
   @brief SLOT for the signal sent when the calibration process ends.
   See gui_calibration_node.cpp
   @param void
   @return void
 */
void MainWindow::calibrationFinished()
{
    std::cout<<"calibrationFinished"<<std::endl;
    ui->bt_stop_nodes->setEnabled(true);
    std::cout<<"stopnodes true"<<std::endl;
    ui->bt_calibrate->setEnabled(true);
    std::cout<<"calibrate true"<<std::endl;
    ui->bt_stop_calibrate->setEnabled(false);
    std::cout<<"stop calibrate true"<<std::endl;
    ui->bt_start_nodes->setEnabled(false);
    std::cout<<"start_nodes true"<<std::endl;
    QMessageBox::information(0, "Calibration Complete", "Sensor calibration has finished.");
    std::cout<<"messagebox true"<<std::endl;
}
