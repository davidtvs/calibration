#include "../include/mainwindow.h"
#include "ui_mainwindow.h"
#include <QProcess>
#include <QDebug>
#include <QtCore>
#include <QtGui>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    QProcess process;
    process.start("gedit", QStringList() << "/home/david/catkin_ws/src/lidar_cam_calibration/calibration_gui/launch/lasers.launch");
    // p.start("echo", QStringList() << "hye");
    bool started = process.waitForStarted();
    qDebug()<< process.readAllStandardOutput() << process.pid() << "started" << started;
    process.waitForFinished();
}

