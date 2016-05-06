#include "calibration_gui/gui_mainwindow.h"
#include "calibration_gui/gui_myrviz.h"

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

    myviz = new MyViz();
    //setCentralWidget(ui->mdiArea);
    ui->mdiArea->addSubWindow(myviz, Qt::FramelessWindowHint); // FramelessWindowHint removes close, minimize and maximize title bar
    myviz->showMaximized();

    ui->treeWidget->setItemDelegate(new MyItemDelegate(ui->treeWidget));

    QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);
    ui->treeWidget->addTopLevelItem(item);
    item->setText(0, parameterBallDiameter);
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable);

    item = new QTreeWidgetItem(ui->treeWidget);
    ui->treeWidget->addTopLevelItem(item);
    item->setText(0, parameterNumPoints);
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable);

    ui->treeWidget->resizeColumnToContents(0);

    ui->bt_stop_nodes->setEnabled(false);
    ui->bt_calibrate->setEnabled(false);

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


void MainWindow::on_bt_add_clicked()
{
    static int rowNumber = 0;
    qDebug() << rowNumber;
    AddRoot(rowNumber);
    rowNumber++;
}

void MainWindow::AddRoot (int rowNumber)
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

    AddChildIP(item, rowNumber);
    //AddChildTopic(item);
}

void MainWindow::AddChildIP (QTreeWidgetItem *parent, int rowNumber)
{
    QString ask_IP = "IP";

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
                ui->bt_remove->setEnabled(false);
            else
                ui->bt_remove->setEnabled(true);
        }
    }
}



void MainWindow::on_bt_start_nodes_clicked()
{
    qDebug() << "in start nodes";

    ui->bt_start_nodes->setEnabled(false);

    processes.clear();
    launchedNodes.clear();

    QList<int> sensorCounter;
    for (int i = 0; i < supportedSensors.size(); i++)
        sensorCounter.push_back(0);

    // i starts at two because comboboxes can only exists after row number 2
    for (int i=2; i < ui->treeWidget->topLevelItemCount(); i++)
    {
        QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i); // Get item containing combobox
        QWidget *widget = ui->treeWidget->itemWidget(item, 0); // ComboBox widget is on column 0
        QString sensor = qobject_cast<QComboBox*>(widget)->currentText(); // Gets ComboBox current text, which is a sensor

        QTreeWidgetItem *itemchild = item->child(0); // Get child item (itemchild) of top level item (item)

        QString sensorIP = "host:=";
        sensorIP += itemchild->text(1); // Gets IP written by user (column 1)

        QString node_name = "node_name:=";

        QString lms151_1 = "true"; // see comment after QStringList arguments

        if (sensor == supportedSensors[0])
        {
            sensorCounter[0] += 1;
            launchedNodes.push_back(supportedSensorsNodes[0] + "_" + QString::number(sensorCounter[0]));
            node_name += launchedNodes.last();
            if (sensorCounter[0] == 2)
                lms151_1="false";
        }
        else if (sensor == supportedSensors[1])
        {
            sensorCounter[1] += 1;
            launchedNodes.push_back(supportedSensorsNodes[1] + "_" + QString::number(sensorCounter[1]));
            node_name += launchedNodes.last();
        }
        else if (sensor == supportedSensors[2])
        {
            sensorCounter[2] += 1;
            launchedNodes.push_back(supportedSensorsNodes[2] + "_" + QString::number(sensorCounter[2]));
            node_name += launchedNodes.last();
        }
        else if (sensor == supportedSensors[3])
        {
            sensorCounter[3] += 1;
            launchedNodes.push_back(supportedSensorsNodes[3] + "_" + QString::number(sensorCounter[3]));
            node_name += launchedNodes.last();
        }
        else
            qDebug() << sensor << "is not on" << supportedSensors;

        QString roslaunch_file = sensor +".launch";

        QString program = "roslaunch";
        QStringList arguments = QStringList() << "calibration_gui"
                                              << roslaunch_file
                                              << sensorIP
                                              << node_name
                                              << "lms151_1:=" + lms151_1;
        /*  The last argument ""lms151_2:=" + lms151" is only needed because subscribing to topics
            on the calibration  and circle detection files is not generalised. SO in order to launch
            multiple LMS151 nodes this workaround is needed - if true a different When that's is done
            the argument should be removed */

        qDebug() << "Launching with:" << program << arguments.join(" ");

        processes.push_back(new QProcess(this));
        connect(processes.last(), SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(NodeFinished(int, QProcess::ExitStatus)));

        processes.last()->start(program, arguments);
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
        qint64 pid = process->pid();
        qDebug() << "Killing process PID:" << pid;
        process->kill();

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
    }
    launchedNodes.clear();
    qDebug() << "All node killed ended" << launchedNodes;

    ui->bt_start_nodes->setEnabled(true);
}

void MainWindow::on_bt_calibrate_clicked()
{
    // start() is a QThread function. It calls QNode::run automatically
    //qnode->start();
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




