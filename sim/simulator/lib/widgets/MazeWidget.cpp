#include <iostream>

#include <QtGui/QPainter>

#include <common/core/AbstractMaze.h>
#include <sim/simulator/lib/widgets/MazeWidget.h>
#include <lib/common/TopicNames.h>
#include <sim/simulator/msgs/msgs.h>

const int MazeWidget::kPaddingPx = 24;
const QBrush MazeWidget::kRobotBrush = QBrush(QColor("#F57C00"));
QBrush MazeWidget::kWallBrush = QBrush(Qt::red);

MazeWidget::MazeWidget() : AbstractTab() {
  setSizePolicy(QSizePolicy::Policy::MinimumExpanding, QSizePolicy::Policy::MinimumExpanding);
  node_.Subscribe(TopicNames::kMaze, &MazeWidget::OnMaze, this);
  node_.Subscribe(TopicNames::kRobotDescription, &MazeWidget::OnRobotDescription, this);
  node_.Subscribe(TopicNames::kRobotSimState, &MazeWidget::OnRobotSimState, this);

  QObject::connect(this,
                   &MazeWidget::MyUpdate,
                   this,
                   static_cast<void (QWidget::*)()>(&QWidget::update), Qt::QueuedConnection);
}

/**
 * All drawing should be done in meters, with the origin as the center of the bottom-left corner.
 * The coordinate system is X increasing across columns, Y increasing across rows, and Z up out of the maze.
 */
void MazeWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  QTransform tf;
  {

    QRect g = this->geometry();

    int w = std::min(g.width(), g.height()) - kPaddingPx;
    double m2p = w / AbstractMaze::MAZE_SIZE_M;

    int ox = 0 + (g.width() - w) / 2;
    int oy = w + (g.height() - w) / 2;

    tf.translate(ox, oy);
    tf = tf.scale(m2p, -m2p);
  }

  // draw the background
  QRectF base = QRectF(0, 0, AbstractMaze::MAZE_SIZE_M, AbstractMaze::MAZE_SIZE_M);
  painter.fillRect(tf.mapRect(base), QApplication::palette().background());

  // Draw the thin-line grid over the whole maze
  for (unsigned int i = 0; i <= AbstractMaze::MAZE_SIZE; i++) {
    QLineF h_line(0, i * AbstractMaze::UNIT_DIST, AbstractMaze::MAZE_SIZE_M, i * AbstractMaze::UNIT_DIST);
    painter.setPen(QApplication::palette().light().color());
    painter.drawLine(tf.map(h_line));

    QLineF v_line((i * AbstractMaze::UNIT_DIST), 0, (i * AbstractMaze::UNIT_DIST), AbstractMaze::MAZE_SIZE_M);
    painter.drawLine(tf.map(v_line));
  }

  // Draw all the walls
  PaintWalls(painter, tf);

  // Draw the mouse
  PaintMouse(painter, tf);
}

void MazeWidget::PaintMouse(QPainter &painter, QTransform tf) {
  QPainterPath footprint;
  footprint.moveTo(mouse_.footprint(0).x(), mouse_.footprint(0).y());
  for (auto pt : mouse_.footprint()) {
    footprint.lineTo(pt.x(), pt.y());
  }

  auto left_wheel = mouse_.left_wheel();
  auto right_wheel = mouse_.right_wheel();
  double lwx = left_wheel.pose().x();
  double lwy = left_wheel.pose().y();
  double rwx = right_wheel.pose().x();
  double rwy = right_wheel.pose().y();
  double lr = left_wheel.radius();
  double lt = left_wheel.thickness();
  double rr = right_wheel.radius();
  double rt = right_wheel.thickness();

  QPainterPath left_wheel_path;
  left_wheel_path.moveTo(lwx - lr, lwy - lt / 2);
  left_wheel_path.lineTo(lwx - lr, lwy + lt / 2);
  left_wheel_path.lineTo(lwx + lr, lwy + lt / 2);
  left_wheel_path.lineTo(lwx + lr, lwy - lt / 2);

  QPainterPath right_wheel_path;
  right_wheel_path.moveTo(rwx - rr, rwy - rt / 2);
  right_wheel_path.lineTo(rwx - rr, rwy + rt / 2);
  right_wheel_path.lineTo(rwx + rr, rwy + rt / 2);
  right_wheel_path.lineTo(rwx + rr, rwy - rt / 2);

  tf.translate(robot_state_.p().x(), robot_state_.p().y());
  tf.rotateRadians(robot_state_.p().theta(), Qt::ZAxis);

  painter.setPen(QPen(Qt::black));
  painter.fillPath(tf.map(footprint), kRobotBrush);
  painter.fillPath(tf.map(left_wheel_path), QBrush(Qt::black));
  painter.fillPath(tf.map(right_wheel_path), QBrush(Qt::black));

  std::vector<std::pair<smartmouse::msgs::XYTheta, double>> sensor_poses;
  sensor_poses.push_back({mouse_.sensors().front().p(), robot_state_.front()});
  sensor_poses.push_back({mouse_.sensors().front_left().p(), robot_state_.front_left()});
  sensor_poses.push_back({mouse_.sensors().front_right().p(), robot_state_.front_right()});
  sensor_poses.push_back({mouse_.sensors().back_left().p(), robot_state_.back_left()});
  sensor_poses.push_back({mouse_.sensors().back_right().p(), robot_state_.back_right()});
  sensor_poses.push_back({mouse_.sensors().gerald_left().p(), robot_state_.gerald_left()});
  sensor_poses.push_back({mouse_.sensors().gerald_right().p(), robot_state_.gerald_right()});

  for (auto pair : sensor_poses) {
    auto sensor_pose = pair.first;
    double sensor_range = pair.second;
    QTransform line_tf(tf);
    line_tf.translate(sensor_pose.x(), sensor_pose.y());
    line_tf.rotateRadians(sensor_pose.theta(), Qt::ZAxis);
    QLineF line(0, 0, sensor_range, 0);
    painter.setPen(QPen(Qt::green));
    painter.drawLine(line_tf.map(line));
  }
}

void MazeWidget::PaintWalls(QPainter &painter, QTransform tf) {
  for (smartmouse::msgs::Wall wall : maze_.walls()) {
    double x1, y1, x2, y2;
    std::tie(x1, y1, x2, y2) = smartmouse::msgs::WallToCoordinates(wall);

    QPainterPath wall_path;
    wall_path.moveTo(x1, y1);
    wall_path.lineTo(x2, y1);
    wall_path.lineTo(x2, y2);
    wall_path.lineTo(x1, y2);
    wall_path.lineTo(x1, y1);
    painter.fillPath(tf.map(wall_path), kWallBrush);
  }
}

const QString MazeWidget::getTabName() {
  return QString("Maze View");
}

void MazeWidget::OnMaze(const smartmouse::msgs::Maze &msg) {
  maze_ = msg;
  emit MyUpdate();
}

void MazeWidget::OnRobotDescription(const smartmouse::msgs::RobotDescription &msg) {
  mouse_ = msg;
  emit MyUpdate();
}

void MazeWidget::OnRobotSimState(const smartmouse::msgs::RobotSimState &msg) {
  robot_state_ = msg;

  emit MyUpdate();
}
