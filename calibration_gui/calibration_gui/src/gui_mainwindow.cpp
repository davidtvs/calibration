#include "calibration_gui/gui_mainwindow.h"
#include "calibration_gui/gui_myrviz.h"

#include <QDebug>
#include <QProcess>
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
    // start() is a QThread function. It calls QNode::run automatically
    qnode->start();
}

/*int MainWindow::treeWidgetFindText (const QString & textToFInd)
{
    QList<QTreeWidgetItem*> items = ui->treeWidget->findItems(textToFInd,
                                    Qt::MatchContains|Qt::MatchRecursive,
                                    0);
    return items[0]->text(1).toInt();
}*/

void MainWindow::on_treeWidget_itemChanged(QTreeWidgetItem *item, int column)
{
    QString changedText = item->text(column); // column should always be 1, but this is more generic
    QString parameter = item->text(0); // get changed paremeter name
    qDebug() << changedText << " " << column << " " << parameter;
    if (parameter == parameterNumPoints)
        qnode->setCalibrationPoints(changedText.toInt());

}


void MainWindow::on_bt_start_nodes_clicked()
{
    qDebug() << "in start nodes";
    QTreeWidgetItem *item = ui->treeWidget->topLevelItem(2);
    QWidget *widget = ui->treeWidget->itemWidget(item, 0);
    QComboBox *combobox = qobject_cast<QComboBox*>(widget);
    qDebug() << combobox->currentText();
}


void MainWindow::setQStrings()
{
    parameterBallDiameter = "Ball Diameter (meters)";
    parameterNumPoints = "No. of Calibration Points";

    supportedSensors = QList<QString>() << "Sick LMS 151_1"
                                        << "Sick LMS 151_2"
                                        << "Sick LD-MRS400001"
                                        << "Point Grey FL3-GE-28S4-C"
                                        << "SwissRanger SR40000";
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

