#include "calibration_gui/gui_mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QProcess>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->treeWidget->setColumnCount(1);
    QList<QTreeWidgetItem *> items;
    for (int i = 0; i < 10; ++i)
       items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("item: %1").arg(i))));
    ui->treeWidget->insertTopLevelItems(0, items);

    QWidget *dualPushButtons = new QWidget();
    QHBoxLayout *hLayout = new QHBoxLayout();
    hLayout->addWidget(new QPushButton("Button1"));
    hLayout->addWidget(new QPushButton("Button2"));
    dualPushButtons->setLayout(hLayout);
    dualPushButtons->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

    ui->treeWidget->setItemWidget(items.value(1),0,dualPushButtons);
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

void MainWindow::on_pushButton_clicked()
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
}
