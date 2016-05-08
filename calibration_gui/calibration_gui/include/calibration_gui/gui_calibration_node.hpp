/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef NODE_HPP_
#define NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
Q_OBJECT

public:
    QNode(int argc, char** argv, const std::string &name );
    virtual ~QNode();

    bool on_init();
    bool on_init(const std::string &master_url, const std::string &host_url);
    void shutdown();
    void run();

    QStringListModel* loggingModel() { return &logging; }
    QPixmap PixmapModel (){return px;}
    const std::string& nodeName() { return node_name; }

    void setCalibrationPoints(int numPoints);
    void setLaunchedNodes(std::vector<std::string> sensors, std::vector<bool> camera);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();

private:
    int num_of_points;
    std::vector<std::string> calibrationNodes;
    std::vector<bool> isCamera;

protected:
    int init_argc;
    char** init_argv;
    QStringListModel logging;
    QPixmap px;
    const std::string node_name;
};

#endif /* NODE_HPP_ */
