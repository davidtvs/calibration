#ifndef GUI_SUPPORTEDSENSORS_H
#define GUI_SUPPORTEDSENSORS_H

#include <QList>
#include <QTreeWidget>

class SupportedSensors
{
public:
    SupportedSensors();

    QList<QString> getSupportedSensors() { return supportedSensors; }

    QList<QString> getSupportedSensorsNodes() { return supportedSensorsNodes; }

    void addTreeChilds(QTreeWidgetItem *parent, const QString sensorID);

    QStringList roslaunchManager(QTreeWidgetItem * item, QString sensor, double ballDiameter);

    QList<QString> getLaunchedNodes() { return launchedNodes; }

    std::vector<bool> getIsCamera() { return isCamera; }

    std::vector<bool> getIsCameraFrame() { return isCameraFrame; }

    void resetLaunchedLists();

private:
    QList<QString> supportedSensors;
    QList<QString> supportedSensorsNodes;
    QList<QString> launchedNodes;
    QList<int> sensorCounter;
    std::vector<bool> isCamera;
    std::vector<bool> isCameraFrame; // needed because swissranger and kinect IR sensor use camera frames but are not cameras

    void makeChild (QTreeWidgetItem *parent, const QString text, int column);

};

#endif // GUI_SUPPORTEDSENSORS_H
