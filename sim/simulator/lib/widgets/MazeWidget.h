#pragma once

#include <sim/simulator/lib/widgets/AbstractTab.h>
#include <QtWidgets>
#include <QtGui/QPaintEvent>
#include <sim/simulator/msgs/robot_sim_state.pb.h>
#include <sim/simulator/msgs/maze.pb.h>
#include <ignition/transport/Node.hh>
#include <msgs/robot_description.pb.h>
#include <msgs/robot_sim_state.pb.h>

class MazeWidget : public AbstractTab {
 Q_OBJECT

 public:
  MazeWidget();

  void OnMaze(const smartmouse::msgs::Maze &msg);
  void OnRobotDescription(const smartmouse::msgs::RobotDescription &msg);
  void OnRobotSimState(const smartmouse::msgs::RobotSimState &msg);

  void paintEvent(QPaintEvent *event);

  const QString getTabName() override;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "NotImplementedFunctions"
 signals:
  void MyUpdate();
#pragma clang diagnostic pop

 private:
  void PaintWalls(QPainter &painter, QTransform tf);
  void PaintMouse(QPainter &painter, QTransform tf);

  static const int kPaddingPx;
  static const QBrush kRobotBrush;
  static QBrush kWallBrush;

  ignition::transport::Node node_;
  smartmouse::msgs::Maze maze_;
  smartmouse::msgs::RobotSimState robot_state_;
  smartmouse::msgs::RobotDescription mouse_;
};

