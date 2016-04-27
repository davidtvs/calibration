#include "calibration_gui/gui_mainwindow.h"
#include <QApplication>

// ====================================================================================
// References:
// http://wiki.ros.org/qt_create/Tutorials/Qt%20App%20Templates?action=show&redirect=qt_ros%252FTutorials%252FQt+App+Templates+-+Electric
// https://github.com/decoderdan/projectPhoenix/wiki/Creating-a-ROS-qt-gui-node
// https://github.com/dxydas/ros-bioloid
// https://github.com/dazhbog/tanky_gui

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
