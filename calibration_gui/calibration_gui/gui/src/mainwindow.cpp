#include "../include/mainwindow.h"
#include "ui_mainwindow.h"
#include <QProcess>
#include <QDebug>
#include <QtCore>
#include <QtGui>

QProcess process;

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

    process.start("gnome-terminal --geometry=50x10-0-10 -x bash -c \"rosrun rviz rviz\" ");

    bool started = process.waitForStarted();
    qDebug()<< process.readAllStandardOutput() << process.pid() << "started" << started;
    process.waitForFinished();

}

