#ifndef MYVIZ_H
#define MYVIZ_H

#include <QWidget>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "MyViz" implements the top level widget for this example.
class MyViz: public QWidget
{
Q_OBJECT
public:
  MyViz( QWidget* parent = 0 );
  virtual ~MyViz();

  void subscribeTopics(const QString qnode_name);

private Q_SLOTS:
  //void setThickness( int thickness_percent );
  //void setCellSize( int cell_size_percent );

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
};
// END_TUTORIAL
#endif // MYVIZ_H
