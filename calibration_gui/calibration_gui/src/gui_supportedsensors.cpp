#include "calibration_gui/gui_supportedsensors.h"

#include <QDebug>

SupportedSensors::SupportedSensors()
{
    supportedSensors = QList<QString>() << "Sick LMS151"
                                        << "Sick LD-MRS400001"
                                        << "Point Grey FL3-GE-28S4-C"
                                        << "SwissRanger SR40000_Ethernet"
                                        << "SwissRanger SR40000_USB";

    supportedSensorsNodes = QList<QString>() << "lms151"
                                             << "ldmrs"
                                             << "pointgrey"
                                             << "swissranger_eth"
                                             << "swissranger_usb";

    for (int i = 0; i < supportedSensors.size(); i++)
        sensorCounter.push_back(0);
}

void SupportedSensors::addTreeChilds(QTreeWidgetItem *item, const QString sensorID)
{
    QString ask_IP = "IP Address";

    if (sensorID == supportedSensors[0]) //Sick LMS151
        makeChild(item, ask_IP, 0);
    else if (sensorID == supportedSensors[1]) //Sick LD-MRS400001
        makeChild(item, ask_IP, 0);
    else if (sensorID == supportedSensors[2]) //Point Grey FL3-GE-28S4-C
        makeChild(item, ask_IP, 0);
    else if (sensorID == supportedSensors[3]) //SwissRanger SR40000_(Ethernet)
        makeChild(item, ask_IP, 0);
    else if (sensorID == supportedSensors[4]) //"SwissRanger SR40000_(USB)"
    {
        // It's only here for consistency. No properties for now.
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

    if (sensor == supportedSensors[0]) // Sick LMS151
    {
        sensorCounter[0] += 1;
        launchedNodes.push_back(supportedSensorsNodes[0] + "_" + QString::number(sensorCounter[0]));
        node_name += launchedNodes.last();

        itemchild = item->child(0); // Get child item (itemchild) of top level item (item)
        sensorIP += itemchild->text(1);

        roslaunch_params << sensorIP << node_name;

        isCamera.push_back(false);
    }
    else if (sensor == supportedSensors[1]) // Sick LD-MRS400001
    {
        sensorCounter[1] += 1;
        launchedNodes.push_back(supportedSensorsNodes[1] + "_" + QString::number(sensorCounter[1]));
        node_name += launchedNodes.last();

        itemchild = item->child(0); // Get child item (itemchild) of top level item (item)
        sensorIP += itemchild->text(1);

        roslaunch_params << sensorIP << node_name;

        isCamera.push_back(false);
    }
    else if (sensor == supportedSensors[2]) // Point Grey FL3-GE-28S4-C
    {
        sensorCounter[2] += 1;
        launchedNodes.push_back(supportedSensorsNodes[2] + "_" + QString::number(sensorCounter[2]));
        node_name += launchedNodes.last();

        itemchild = item->child(0); // Get child item (itemchild) of top level item (item)
        sensorIP += itemchild->text(1);

        roslaunch_params << sensorIP << node_name;

        isCamera.push_back(true);
    }
    else if (sensor == supportedSensors[3]) // SwissRanger SR40000_(Ethernet)
    {
        sensorCounter[3] += 1;
        launchedNodes.push_back(supportedSensorsNodes[3] + "_" + QString::number(sensorCounter[3]));
        node_name += launchedNodes.last();

        itemchild = item->child(0); // Get child item (itemchild) of top level item (item)
        sensorIP += itemchild->text(1);

        roslaunch_params << sensorIP << node_name;

        isCamera.push_back(false);
    }
    else if (sensor == supportedSensors[4]) // SwissRanger SR40000_(USB)
    {
        sensorCounter[4] += 1;
        launchedNodes.push_back(supportedSensorsNodes[4] + "_" + QString::number(sensorCounter[4]));
        node_name += launchedNodes.last();

        // No childs

        roslaunch_params << node_name;

        isCamera.push_back(false);
    }
    else
        qDebug() << sensor << "is not on" << supportedSensors;


    QString roslaunch_file = sensor +".launch";

    QStringList roslaunch_arg = QStringList() << "calibration_gui"
                                              << roslaunch_file
                                              << roslaunch_params;

    return roslaunch_arg;
}


void SupportedSensors::resetLaunchedLists()
{
    isCamera.clear();
    launchedNodes.clear();
    sensorCounter.clear();
    for (int i = 0; i < supportedSensors.size(); i++)
        sensorCounter.push_back(0);
}
