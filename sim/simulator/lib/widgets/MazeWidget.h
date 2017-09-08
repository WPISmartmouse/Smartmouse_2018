#pragma once

#include <QtWidgets>
#include <QtGui/QPaintEvent>
#include <sim/simulator/msgs/robot_ui_state.pb.h>
#include <sim/simulator/msgs/maze.pb.h>
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

  static const int kPaddingPx;
  static const QBrush kRobotBrush;
  static QBrush kWallBrush;

  ignition::transport::Node node_;
  smartmouse::msgs::Maze maze_;
  QPainterPath footprint_;
  smartmouse::msgs::RobotUiState robot_state_;
};

