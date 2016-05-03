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
  // Construct and lay out labels and slider controls.
  /*QLabel* thickness_label = new QLabel( "Line Thickness" );
  QSlider* thickness_slider = new QSlider( Qt::Horizontal );
  thickness_slider->setMinimum( 1 );
  thickness_slider->setMaximum( 100 );
  QLabel* cell_size_label = new QLabel( "Cell Size" );
  QSlider* cell_size_slider = new QSlider( Qt::Horizontal );
  cell_size_slider->setMinimum( 1 );
  cell_size_slider->setMaximum( 100 );
  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( thickness_label, 0, 0 );
  controls_layout->addWidget( thickness_slider, 0, 1 );
  controls_layout->addWidget( cell_size_label, 1, 0 );
  controls_layout->addWidget( cell_size_slider, 1, 1 );*/

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  //main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Make signal/slot connections.
  //connect( thickness_slider, SIGNAL( valueChanged( int )), this, SLOT( setThickness( int )));
  //connect( cell_size_slider, SIGNAL( valueChanged( int )), this, SLOT( setCellSize( int )));

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

  // Configure the GridDisplay the way we like it.
  //grid_->subProp( "Line Style" )->setValue( "Billboards" );
  grid_->subProp( "Color" )->setValue( Qt::white);

  // Initialize the slider values.
  //thickness_slider->setValue( 1 );
  //cell_size_slider->setValue( 1 );

  markerarray_= manager_->createDisplay( "rviz/Marker", "Marker", true );

  ROS_ASSERT( markerarray_ != NULL );

  markerarray_->subProp( "Marker Topic" )->setValue( "/ATLASCAR1" );
  //markerarray_->setTopic("/ATLASCAR1", "visualization_msgs::Marker::MESH_RESOURCE");
}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the line thickness of the grid by changing the
// grid's "Line Width" property.
/*void MyViz::setThickness( int thickness_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
  }
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void MyViz::setCellSize( int cell_size_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
  }
}*/
