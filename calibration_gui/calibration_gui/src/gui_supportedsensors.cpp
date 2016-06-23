#include "calibration_gui/gui_supportedsensors.h"

#include <QDebug>

SupportedSensors::SupportedSensors()
{
    // Add supported sensors: (roslaunch file name, node name, is it a camera?)
    this->addSupportedSensors("Sick LMS151", "lms151", false);
    this->addSupportedSensors("Sick LD-MRS400001", "ldmrs", false);
    this->addSupportedSensors("Point Grey FL3-GE-28S4-C", "pointgrey", true);
    this->addSupportedSensors("SwissRanger SR4000_Ethernet", "swissranger_eth", false);
    this->addSupportedSensors("SwissRanger SR4000_USB", "swissranger_usb", false);
    this->addSupportedSensors("Microsoft Kinect 3D Sensor", "kinect_3d", false);

    for (int i = 0; i < supportedSensors.size(); i++)
        sensorCounter.push_back(0);
}

void SupportedSensors::addSupportedSensors (QString roslaunch_name, QString node_name, bool camera)
{
    supportedSensors.push_back(roslaunch_name);
    supportedSensorsNodes.push_back(node_name);
    supportedCamera.push_back(camera);
}

void SupportedSensors::addTreeChilds(QTreeWidgetItem *item, const QString sensorID)
{
    QString ask_IP = "IP Address";

    if (sensorID == supportedSensors[0]) //Sick LMS151
        makeChild(item, ask_IP, 0);
    else if (sensorID == supportedSensors[1]) //Sick LD-MRS400001
        makeChild(item, ask_IP, 0);
    else if (sensorID == supportedSensors[2]) //Point Grey FL3-GE-28S4-C
    {
        // Nothing for now...IP is changed through FlyCap2 software
    }
    else if (sensorID == supportedSensors[3]) //SwissRanger SR4000_(Ethernet)
        makeChild(item, ask_IP, 0);
    else if (sensorID == supportedSensors[4]) // SwissRanger SR4000_(USB)
    {
        // Nothing for now
    }
    else if (sensorID == supportedSensors[5]) // Microsoft Kinect IR Sensor
    {
        // Nothing for now
    }
    else
    {
        // Alert the user because sensorID is unexpected
    }
}

void SupportedSensors::makeChild (QTreeWidgetItem *parent, const QString text, int column)
{
    QTreeWidgetItem *item = new QTreeWidgetItem(parent);

    parent->addChild(item);

    // Source https://forum.qt.io/topic/28360/clicking-in-qtreewidget-item-while-already-editing-reopens-the-editor/5
    // Makes this item editable
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable);

    item->setText(column, text);

    item->parent()->setExpanded(true);
}



// ====================================================================================
// References:
// http://stackoverflow.com/questions/36156519/set-qstring-to-qprocess-properly
// http://stackoverflow.com/questions/24771293/how-to-get-parameter-from-ros-launch-file-and-use-it-in-qt
// http://stackoverflow.com/questions/10098980/real-time-display-of-qprocess-output-in-a-textbrowser
// http://doc.qt.io/qt-4.8/qprocess.html
// ====================================================================================

QStringList SupportedSensors::roslaunchManager(QTreeWidgetItem * item, QString sensor, double ballDiameter)
{
    QString ball_diameter = "ball_diameter:=" + QString::number(ballDiameter);
    QString node_name = "node_name:=";
    QString sensorIP = "host:=";

    QTreeWidgetItem *itemchild;

    QStringList roslaunch_params = QStringList() << ball_diameter;

    for (int i = 0; i<supportedSensors.size(); i++)
    {
        if (sensor == supportedSensors[i])
        {
            sensorCounter[i] += 1;
            launchedNodes.push_back(supportedSensorsNodes[i] + "_" + QString::number(sensorCounter[i]));
            node_name += launchedNodes.last();

            if (item->childCount())
            {
                itemchild = item->child(0);
                sensorIP += itemchild->text(1);
            }
            roslaunch_params << sensorIP << node_name;
            isCamera.push_back(supportedCamera[i]);
        }
    }

    QString roslaunch_file = sensor +".launch";

    QStringList roslaunch_arg = QStringList() << "calibration_gui"
                                              << roslaunch_file
                                              << roslaunch_params; 

    return roslaunch_arg;
}


std::vector<std::string> SupportedSensors::getDisplayNames()
{
    std::vector<std::string> displayNames;
    for (int i = 0; i < launchedNodes.size(); i++)
    {
        if (isCamera[i])
        {
            displayNames.push_back(launchedNodes[i].toStdString() + "_rigid");
            displayNames.push_back(launchedNodes[i].toStdString() + "_pnp");
        }
        else
            displayNames.push_back(launchedNodes[i].toStdString());
    }
    return displayNames;
}


void SupportedSensors::resetLaunchedLists()
{
    isCamera.clear();
    launchedNodes.clear();
    sensorCounter.clear();
    for (int i = 0; i < supportedSensors.size(); i++)
        sensorCounter.push_back(0);
}


