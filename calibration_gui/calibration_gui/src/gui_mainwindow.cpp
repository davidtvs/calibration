#include "../include/calibration_gui/gui_mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QProcess>
#include <QLineEdit>
#include <QToolButton>
#include <QAbstractButton> // ToolButton setText
#include <QComboBox>
#include <QCheckBox>
#include <QHeaderView>
#include <QFileDialog>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QString style = "QTreeWidget::item:!selected "
      "{ "
        "border: 1px solid gainsboro; "
        "border-left: none; "
        "border-top: none; "
        "border-bottom: none; "
      "}"
      "QTreeWidget::item:selected {}";
      ui->treeWidget->setStyleSheet(style);
}

MainWindow::~MainWindow()
{
    delete ui;
}

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


void MainWindow::on_bt_add_clicked()
{
    static int rowNumber = 0;
    qDebug() << rowNumber;
    AddRoot(rowNumber);
    rowNumber++;
    //ui->treeWidget->setItemDelegate(new GridDelegate(ui->treeWidget));
}

void MainWindow::AddRoot (int rowNumber)
{
    // ComboBox with avaiable lasers and cameras
    QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);

    ui->treeWidget->addTopLevelItem(item);

    QComboBox *deviceComboBox = new QComboBox();
    deviceComboBox->addItem("Sick LMS 151");
    deviceComboBox->addItem("Sick LD-MRS400001");
    deviceComboBox->addItem("Point Grey FL3-GE-28S4-C");
    deviceComboBox->addItem("SwissRanger SR40000");

    ui->treeWidget->setItemWidget(item,0,deviceComboBox);

    // CheckBox so the user can chose to calibrate the device or not
    QCheckBox *deviceCheckBox = new QCheckBox();
    deviceCheckBox->setCheckState(Qt::Checked);
    ui->treeWidget->setItemWidget(item,1,deviceCheckBox);

    AddChildRoslaunch(item, rowNumber);
    AddChildTopic(item);
}

void MainWindow::AddChildRoslaunch (QTreeWidgetItem *parent, int rowNumber)
{
    QString descr_roslaunch = "Roslauch File";

    QTreeWidgetItem *item = new QTreeWidgetItem(parent);

    parent->addChild(item);

    item->setText(0, descr_roslaunch);

    QWidget *dualPushButtons = new QWidget();
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

    ui->treeWidget->setItemWidget(item,1,dualPushButtons);
}

void MainWindow::AddChildTopic (QTreeWidgetItem *parent)
{
    QString descr_topic = "Topic";
    QTreeWidgetItem *item = new QTreeWidgetItem(parent);

    parent->addChild(item);

    item->setText(0, descr_topic);
}

void MainWindow::on_bt_remove_clicked()
{

}

void MainWindow::toolButton_clicked(int rowNumber)
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Open File"));
    QPoint point = QPoint(1, 2);
    QTreeWidgetItem *treeWidgetItem = ui->treeWidget->itemAt(1, rowNumber);

     QTreeWidgetItem * line = treeWidgetItem->child(0);

     line->setText(0, "TESSATYSTAY");

    qDebug() << filename << "rowNumber = " << rowNumber;
}
