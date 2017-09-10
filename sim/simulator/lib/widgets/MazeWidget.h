#pragma once

#include <QtGui/QPaintEvent>
#include <QGraphicsView>
#include <sim/simulator/msgs/robot_ui_state.pb.h>
#include <sim/simulator/msgs/maze.pb.h>
#include <ignition/transport/Node.hh>
#include <msgs/robot_description.pb.h>
#include <msgs/robot_sim_state.pb.h>

class MazeWidget : public QGraphicsView {
 Q_OBJECT

 public:
  MazeWidget();

  void OnMaze(const smartmouse::msgs::Maze &msg);
  void OnRobotDescription(const smartmouse::msgs::RobotDescription &msg);
  void OnRobotSimState(const smartmouse::msgs::RobotSimState &msg);

  const QString getTabName();

 protected:
  void resizeEvent(QResizeEvent *event) override;

 private:
  void PaintWalls(QPainter &painter, QTransform tf);
  void PaintMouse(QPainter &painter, QTransform tf);

  static const int kPaddingPx;
  static const QBrush kRobotBrush;
  static QBrush kWallBrush;

  ignition::transport::Node node_;
  smartmouse::msgs::Maze maze_;
  smartmouse::msgs::RobotUiState robot_state_;
  smartmouse::msgs::RobotDescription mouse_;

  QGraphicsScene *graphics_scene_;
  QGraphicsRectItem *background_rect_;
};

