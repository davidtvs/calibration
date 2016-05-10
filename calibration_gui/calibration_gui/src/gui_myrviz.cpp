#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QDebug>
#include <QVariant>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "calibration_gui/gui_myrviz.h"

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QWidget( parent )
{

  // Construct render panel.
  render_panel_ = new rviz::RenderPanel();
  // Layout
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );

  manager_->setFixedFrame( "/my_frame3" );

  manager_->initialize();
  manager_->startUpdate();

  // Create a Grid display.
  grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
  ROS_ASSERT( grid_ != NULL );

  markerarray_= manager_->createDisplay( "rviz/Marker", "Marker", true );

  ROS_ASSERT( markerarray_ != NULL );

}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

void MyViz::subscribeTopics(const QString qnode_name)
{
    QString calibPointsTopic = "/" + qnode_name + "/CalibrationPoints";
    qDebug() << "Calibration points topic:" + calibPointsTopic;
    QString Model3DTopic = "/" + qnode_name + "/3DModel";
    qDebug() << "Calibration points topic:" + Model3DTopic;


    markerarray_= manager_->createDisplay( "rviz/MarkerArray", "MarkerArray", true );
    markerarray_->subProp( "Marker Topic" )->setValue(calibPointsTopic);

    markerarray_->subProp( "Marker Topic" )->setValue( Model3DTopic );
}
