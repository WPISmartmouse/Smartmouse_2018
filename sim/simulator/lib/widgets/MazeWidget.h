#pragma once

#include <QtWidgets>
#include <QtGui/QPaintEvent>
#include <msgs/physics_config.pb.h>
#include <msgs/maze.pb.h>
#include <ignition/transport/Node.hh>

class MazeWidget : public QWidget {
 Q_OBJECT

 public:
  MazeWidget();

  void OnMaze(const smartmouse::msgs::Maze &msg);

  void paintEvent(QPaintEvent *event);

  const QString getTabName();

 private:
  QRectF PaintWall(smartmouse::msgs::Wall wall);

  static const int PADDING_PX;
  static QBrush wallBrush;

  ignition::transport::Node node_;
  smartmouse::msgs::Maze maze_;
};

