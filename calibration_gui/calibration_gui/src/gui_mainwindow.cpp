#include "calibration_gui/gui_mainwindow.h"
#include "calibration_gui/gui_options.h"

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

MyViz* myviz;

MainWindow::MainWindow(QNode *node, QWidget *parent)
    :QMainWindow(parent)
    ,ui(new Ui::MainWindow)
    ,qnode(node)
{
    ui->setupUi(this);

    setQStrings();

    qnode->on_init();

    mRviz = new MyViz();
    ui->mdiArea_calibration->addSubWindow(mRviz, Qt::FramelessWindowHint); // FramelessWindowHint removes close, minimize and maximize title bar
    mRviz->showMaximized();

    ui->treeWidget->headerItem()->setText(0, "Calibration Manager");
    ui->treeWidget->headerItem()->setText(1, "");
    ui->treeWidget->setItemDelegate(new MyItemDelegate(ui->treeWidget));

    //addCalibOptions(); // adds calibration options (ball diameter, number of points...)

    AddRoot();

    ui->treeWidget->resizeColumnToContents(0);

    ui->bt_stop_nodes->setEnabled(false);
    ui->bt_calibrate->setEnabled(false);

    connect(ui->bt_options, SIGNAL(clicked()), this, SLOT(on_actionOptions_triggered()));

    // Draw vertical bar between columns
    /*QString style = "QTreeWidget::item:!selected "
      "{ "
        "border: 1px solid gainsboro; "
        "border-left: none; "
        "border-top: none; "
        "border-bottom: none; "
      "}"
      "QTreeWidget::item:selected {}";
      ui->treeWidget->setStyleSheet(style);*/
}

MainWindow::~MainWindow()
{
    delete myviz;
    delete ui;
}


void MainWindow::addCalibOptions()
{
    QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);
    ui->treeWidget->addTopLevelItem(item);
    item->setText(0, "Calibration Options");

    QTreeWidgetItem *childitem = new QTreeWidgetItem(item);
    childitem->setText(0, parameterBallDiameter);
    childitem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable);
    item->addChild(childitem);

    childitem = new QTreeWidgetItem(item);
    childitem->setText(0, parameterNumPoints);
    childitem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable);
    item->addChild(childitem);

    item->setExpanded(true);
}

void MainWindow::on_bt_add_clicked()
{
    AddRoot();
}


void MainWindow::AddRoot ()
{
    // ComboBox with avaiable lasers and cameras
    QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);

    ui->treeWidget->addTopLevelItem(item);

    QComboBox *deviceComboBox = new QComboBox();
    deviceComboBox->addItems(supportedSensors);

    ui->treeWidget->setItemWidget(item,0,deviceComboBox);

    // CheckBox so the user can chose to calibrate the device or not
    QCheckBox *deviceCheckBox = new QCheckBox();
    deviceCheckBox->setCheckState(Qt::Checked);
    ui->treeWidget->setItemWidget(item,1,deviceCheckBox);

    AddChildIP(item);
    //AddChildTopic(item);
}

void MainWindow::AddChildIP (QTreeWidgetItem *parent)
{
    QString ask_IP = "IP Address";

    QTreeWidgetItem *item = new QTreeWidgetItem(parent);

    parent->addChild(item);

    // Source https://forum.qt.io/topic/28360/clicking-in-qtreewidget-item-while-already-editing-reopens-the-editor/5
    // Makes this item editable
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable);

    item->setText(0, ask_IP);

    item->parent()->setExpanded(true);

    /*QWidget *dualPushButtons = new QWidget();
    QHBoxLayout *hLayout = new QHBoxLayout();

    QLineEdit* lineEdit = new QLineEdit("Button1");
    lineEdit->setFrame(false);

    QToolButton* toolButton = new QToolButton();
    toolButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    toolButton->setText("...");


    QSignalMapper * ButtonSignalMapper = new QSignalMapper(this);
    connect(ButtonSignalMapper, SIGNAL(mapped(int)), this, SLOT(toolButton_clicked(int)));

    ButtonSignalMapper->setMapping(toolButton, rowNumber);

    connect(toolButton, SIGNAL(clicked()), ButtonSignalMapper, SLOT(map()));

    //connect(ui->treeWidget, SIGNAL())

    hLayout->addWidget(lineEdit);
    hLayout->addWidget(toolButton);

    dualPushButtons->setLayout(hLayout);

    ui->treeWidget->setItemWidget(item,1,dualPushButtons);*/
}

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


void MainWindow::on_treeWidget_itemChanged(QTreeWidgetItem *item, int column)
{
    qDebug() << "on_treeWidget_itemChanged";
    QString changedText = item->text(column); // column should always be 1, but this is more generic
    QString parameter = item->text(0); // get changed paremeter name
    qDebug() << changedText << " " << column << " " << parameter;
    if (parameter == parameterNumPoints)
        qnode->setCalibrationPoints(changedText.toInt());

}

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


void MainWindow::on_bt_start_nodes_clicked()
{
    qDebug() << "in start nodes";

    ui->bt_start_nodes->setEnabled(false);

    processes.clear();
    launchedNodes.clear();
    isCamera.clear();

    QList<int> sensorCounter;
    for (int i = 0; i < supportedSensors.size(); i++)
        sensorCounter.push_back(0);

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

            QTreeWidgetItem *itemchild = item->child(0); // Get child item (itemchild) of top level item (item)

            QString sensorIP = "host:=";
            sensorIP += itemchild->text(1); // Gets IP written by user (column 1)

            QString node_name = "node_name:=";

            if (sensor == supportedSensors[0]) // lms151
            {
                sensorCounter[0] += 1;
                launchedNodes.push_back(supportedSensorsNodes[0] + "_" + QString::number(sensorCounter[0]));
                node_name += launchedNodes.last();
                isCamera.push_back(false);
            }
            else if (sensor == supportedSensors[1]) //ldmrs
            {
                sensorCounter[1] += 1;
                launchedNodes.push_back(supportedSensorsNodes[1] + "_" + QString::number(sensorCounter[1]));
                node_name += launchedNodes.last();
                isCamera.push_back(false);
            }
            else if (sensor == supportedSensors[2]) // pointgrey
            {
                sensorCounter[2] += 1;
                launchedNodes.push_back(supportedSensorsNodes[2] + "_" + QString::number(sensorCounter[2]));
                node_name += launchedNodes.last();
                isCamera.push_back(true);
            }
            else if (sensor == supportedSensors[3]) // swissranger
            {
                sensorCounter[3] += 1;
                launchedNodes.push_back(supportedSensorsNodes[3] + "_" + QString::number(sensorCounter[3]));
                node_name += launchedNodes.last();
                isCamera.push_back(false);
            }
            else
                qDebug() << sensor << "is not on" << supportedSensors;


            QString roslaunch_file = sensor +".launch";

            QString program = "roslaunch";
            QStringList arguments = QStringList() << "calibration_gui"
                                                  << roslaunch_file
                                                  << sensorIP
                                                  << node_name;

            qDebug() << "Launching with:" << program << arguments.join(" ");

            processes.push_back(new QProcess(this));
            connect(processes.last(), SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(NodeFinished(int, QProcess::ExitStatus)));

            processes.last()->start(program, arguments);
        }
    }

    ui->bt_stop_nodes->setEnabled(true);
    ui->bt_calibrate->setEnabled(true);

}

void MainWindow::on_bt_stop_nodes_clicked()
{
    qDebug() << "Nodes to be killed" << launchedNodes;

    ui->bt_stop_nodes->setEnabled(false);
    ui->bt_calibrate->setEnabled(false);

    foreach (QProcess *process, processes)
    {
        QString nodeName = "/" + launchedNodes.first() + "/" + launchedNodes.first();
        QProcess nodeKiller;
        nodeKiller.start("rosnode", QStringList() << "kill" << nodeName);
        if (nodeKiller.waitForStarted(-1))
        {
            while(nodeKiller.waitForReadyRead(-1))
                qDebug() <<  nodeKiller.readAllStandardOutput();
        }
        launchedNodes.removeFirst();
        qDebug() << "Killed node:" << nodeName;
        qDebug() << "Nodes remaining:" << launchedNodes;

        /*qint64 pid = process->pid();
        qDebug() << "Killing process PID:" << pid;
        process->kill();*/
    }
    launchedNodes.clear();
    qDebug() << "All node killed ended" << launchedNodes;
}

void MainWindow::on_bt_calibrate_clicked()
{
    std::vector<std::string> vec;

    foreach( QString str, launchedNodes) {
        vec.push_back(str.toStdString());
    }

    qnode->setLaunchedNodes(vec, isCamera);

    // start() is a QThread function. It calls QNode::run automatically
    qnode->start();

    QString qnode_name = QString::fromStdString(qnode->nodeName());
    mRviz->subscribeTopics(qnode_name);
}


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
        ui->bt_stop_nodes->setEnabled(false);
        ui->bt_calibrate->setEnabled(false);
        ui->bt_start_nodes->setEnabled(true);
    }
    qDebug() << processes;
}


void MainWindow::on_actionOptions_triggered()
{
    Options mOptions;
    mOptions.setModal(true);
    mOptions.exec();
}

/*void MainWindow::on_tabWidget_currentChanged(int index)
{
    qDebug() << "on_tabWidget_currentChanged";

    ui->mdiArea_lasers->closeActiveSubWindow();
    ui->mdiArea_cameras->closeActiveSubWindow();
    ui->mdiArea_calib->closeActiveSubWindow();

    if (index == 0)
    {
        ui->mdiArea_lasers->addSubWindow(mRviz, Qt::FramelessWindowHint); // FramelessWindowHint removes close, minimize and maximize title bar
        mRviz->showMaximized();
    }
    else if (index == 1)
    {
        ui->mdiArea_cameras->addSubWindow(mRviz, Qt::FramelessWindowHint); // FramelessWindowHint removes close, minimize and maximize title bar
        mRviz->showMaximized();
    }
    else
    {
        ui->mdiArea_calib->addSubWindow(mRviz, Qt::FramelessWindowHint); // FramelessWindowHint removes close, minimize and maximize title bar
        mRviz->showMaximized();
    }

}*/

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
    }
}


void MainWindow::setQStrings()
{
    parameterBallDiameter = "Ball Diameter (meters)";
    parameterNumPoints = "No. of Calibration Points";

    supportedSensors = QList<QString>() << "Sick LMS 151"
                                        << "Sick LD-MRS400001"
                                        << "Point Grey FL3-GE-28S4-C"
                                        << "SwissRanger SR40000";

    supportedSensorsNodes = QList<QString>() << "lms151"
                                             << "ldmrs"
                                             << "pointgrey"
                                             << "swissranger";
}



/*void MainWindow::AddChildTopic (QTreeWidgetItem *parent)
{
    QString descr_topic = "Topic";
    QTreeWidgetItem *item = new QTreeWidgetItem(parent);

    parent->addChild(item);

    item->setText(0, descr_topic);
}*/


/*void MainWindow::toolButton_clicked(int rowNumber)
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Open File"));
    QPoint point = QPoint(1, 2);
    QTreeWidgetItem *treeWidgetItem = ui->treeWidget->itemAt(1, rowNumber);

     QTreeWidgetItem * line = treeWidgetItem->child(0);

     line->setText(0, "TESSATYSTAY");

    qDebug() << filename << "rowNumber = " << rowNumber;
}*/


// ====================================================================================
// References:
// http://stackoverflow.com/questions/36156519/set-qstring-to-qprocess-properly
// http://stackoverflow.com/questions/24771293/how-to-get-parameter-from-ros-launch-file-and-use-it-in-qt
// http://stackoverflow.com/questions/10098980/real-time-display-of-qprocess-output-in-a-textbrowser
// http://doc.qt.io/qt-4.8/qprocess.html
// ====================================================================================

/*void MainWindow::on_bt_add_clicked()
{
    QProcess process;
    QString program = "roslaunch";
    QStringList arguments;
    arguments << "calibration_gui" << "lasers.launch";
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    process.setProcessEnvironment(env);
    process.start(program, arguments);

    qDebug()<< "PID: " << process.pid();
    int count = 0;
    if (process.waitForStarted(-1)) {
        while(process.waitForReadyRead(-1)) {
            qDebug() <<  process.readAllStandardOutput();
        }
    }
    qDebug() << "Process ended";
}*/
