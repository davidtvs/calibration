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
#include <QMessageBox>
#include <QMutexLocker>
    #include <QWaitCondition>

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

    void setCalibrationPoints(const int numPoints) { num_of_points = numPoints; }
    void setMinDistance (const double distance) { min_distance = distance; }
    void setMaxDisplacement(const double distance) { max_displacement = distance; }
    void setLaunchedNodes(const std::vector<std::string> nodes, const std::vector<bool> camera, const std::vector<bool> cameraFrame);
    void setAutoAcquisition(const bool acquisition_type) { acquisitionIsAuto = acquisition_type; }
    void setDoCalibration(const bool calibration_state) { doCalibration = calibration_state; }

signals:
    void loggingUpdated();
    void calibrationComplete();
    void showMsg( const QString& msg, QMessageBox::StandardButton* answer);

private slots:
    void msgShower(const QString& msg, QMessageBox::StandardButton* answer);

private:
    int num_of_points;
    double min_distance;
    double max_displacement;
    bool acquisitionIsAuto;
    bool doCalibration;

    std::vector<std::string> calibrationNodes;
    std::vector<bool> isCamera;
    std::vector<bool> isCameraFrame;


    QWaitCondition waitCondition;
    QMutex mutex;

protected:
    int init_argc;
    char** init_argv;
    QStringListModel logging;
    QPixmap px;
    const std::string node_name;
};

#endif /* NODE_HPP_ */
