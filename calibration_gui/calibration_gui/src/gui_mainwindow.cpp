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

MainWindow::MainWindow(QNode *node, QWidget *parent)
    :QMainWindow(parent)
    ,ui(new Ui::MainWindow)
    ,qnode(node)
{
    ui->setupUi(this);

    // Start Calibration node
    qnode->on_init();

    // Initialise classes
    mRviz = new MyViz();
    mOptions = new Options();
    mSensors = new SupportedSensors();
    mProgress = new QProgressIndicator();

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

    // Disable buttons
    ui->bt_stop_nodes->setEnabled(false);
    ui->bt_calibrate->setEnabled(false);

    // Connecting signals to slots
    connect(ui->bt_options, SIGNAL(clicked()), this,
            SLOT(on_actionOptions_triggered())); // Options button
    connect(deviceComboBox, SIGNAL(currentIndexChanged(QString)), this,
            SLOT(combobox_itemChanged(QString))); // Combobox
}

MainWindow::~MainWindow()
{
    delete mRviz;
    delete ui;
}




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
    connect(deviceComboBox, SIGNAL(currentIndexChanged(QString)), this, SLOT(combobox_itemChanged(QString)));


    // Add root's childs

    /* ComoboBox by default selects the first sensor on it's list (index 0),
     * so we add the childs of the first item on the supportedSensors list
     */
    mSensors->addTreeChilds(item, supportedSensors[0]);


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

void MainWindow::on_actionOptions_triggered()
{
    mOptions->setModal(true);
    mOptions->exec();


}


void MainWindow::on_treeWidget_itemChanged(QTreeWidgetItem *item, int column)
{
    qDebug() << "on_treeWidget_itemChanged";
    QString changedText = item->text(column); // column should always be 1, but this is more generic
    QString parameter = item->text(0); // get changed paremeter name
    qDebug() << changedText << " " << column << " " << parameter;
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


void MainWindow::on_bt_start_nodes_clicked()
{
    qDebug() << "in start nodes";

    ui->bt_start_nodes->setEnabled(false);

    int ballDiameter = mOptions->getBallDiameter().toInt();
    mSensors->setBallDiameter(ballDiameter);

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

            arguments = mSensors->roslaunchManager(item, sensor);

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

void MainWindow::on_bt_stop_nodes_clicked()
{
    qDebug() << "Nodes to be killed" << launchedNodes;

    ui->bt_stop_nodes->setEnabled(false);
    ui->bt_calibrate->setEnabled(false);

    ui->horizontalLayout->insertWidget(0, mProgress);

    mProgress->startAnimation();

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
        qDebug() << "Kill order to node:" << nodeName;
        qDebug() << "Nodes remaining:" << launchedNodes;
    }
    qDebug() << "Sent kill orders to all nodes" << launchedNodes;
}

void MainWindow::on_bt_calibrate_clicked()
{
    std::vector<std::string> vec;

    foreach( QString str, launchedNodes) {
        vec.push_back(str.toStdString());
    }

    qnode->setLaunchedNodes(vec, isCamera);

    int num_calib_points = mOptions->getNumCalibPoints().toInt();
    qnode->setCalibrationPoints(num_calib_points);
    int min_distance = mOptions->getMinDistance().toInt();
    qnode->setMinDistance(min_distance);

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
        launchedNodes.clear();
        isCamera.clear();
        mSensors->resetLaunchedLists();

        ui->bt_stop_nodes->setEnabled(false);
        ui->bt_calibrate->setEnabled(false);
        ui->bt_start_nodes->setEnabled(true);

        if (mProgress->isAnimated())
        {
            ui->horizontalLayout->removeWidget(mProgress);
        }
    }
    qDebug() << processes;
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
