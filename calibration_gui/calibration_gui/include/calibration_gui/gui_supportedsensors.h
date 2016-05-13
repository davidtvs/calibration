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

    void resetLaunchedLists();

private:
    QList<QString> supportedSensors;
    QList<QString> supportedSensorsNodes;
    QList<QString> launchedNodes;
    QList<int> sensorCounter;
    std::vector<bool> isCamera;

    void makeChild (QTreeWidgetItem *parent, const QString text, int column);

};

#endif // GUI_SUPPORTEDSENSORS_H
