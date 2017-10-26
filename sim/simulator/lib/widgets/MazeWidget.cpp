#include <iostream>

#include <QtGui/QPainter>

#include <sim/simulator/lib/widgets/MazeWidget.h>
#include <lib/common/TopicNames.h>

const int MazeWidget::kPaddingPx = 24;
const QBrush MazeWidget::kRobotBrush = QBrush(QColor("#F57C00"));
QBrush MazeWidget::kWallBrush = QBrush(Qt::red);

MazeWidget::MazeWidget() : AbstractTab(), mouse_set_(false) {
  setSizePolicy(QSizePolicy::Policy::MinimumExpanding, QSizePolicy::Policy::MinimumExpanding);
  node_.Subscribe(TopicNames::kMaze, &MazeWidget::OnMaze, this);
  node_.Subscribe(TopicNames::kRobotDescription, &MazeWidget::OnRobotDescription, this);
  node_.Subscribe(TopicNames::kRobotSimState, &MazeWidget::OnRobotSimState, this);

  QObject::connect(this,
                   &MazeWidget::MyUpdate,
                   this,
                   static_cast<void (QWidget::*)()>(&QWidget::update), Qt::QueuedConnection);
}

void MazeWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  QTransform tf;
  {

    QRect g = this->geometry();

    int w = std::min(g.width(), g.height()) - kPaddingPx;
    double cell_units_to_pixels = w / smartmouse::maze::SIZE_CU;

    int origin_col = (g.width() - w) / 2;
    int origin_row = (g.height() - w) / 2;

    tf.translate(origin_col, origin_row);
    tf = tf.scale(cell_units_to_pixels, cell_units_to_pixels);
  }

  // draw the background
  QRectF base = QRectF(0, 0, smartmouse::maze::SIZE_CU, smartmouse::maze::SIZE_CU);
  painter.fillRect(tf.mapRect(base), QApplication::palette().background());

  // Draw the thin-line grid over the whole maze
  for (unsigned int i = 0; i <= smartmouse::maze::SIZE; i++) {
    QLineF h_line(0, i, smartmouse::maze::SIZE_CU, i);
    painter.setPen(QApplication::palette().light().color());
    painter.drawLine(tf.map(h_line));

    QLineF v_line(i, 0, i, smartmouse::maze::SIZE_CU);
    painter.drawLine(tf.map(v_line));
  }

  // Draw all the walls
  PaintWalls(painter, tf);

  // Draw the mouse
  if (mouse_set_) {
    PaintMouse(painter, tf);
  }
}

void MazeWidget::PaintMouse(QPainter &painter, QTransform tf) {
  QPainterPath footprint;
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

  tf.translate(robot_state_.p().col(), robot_state_.p().row());
  tf.rotateRadians(robot_state_.p().theta(), Qt::ZAxis);
  tf.scale(1 / smartmouse::maze::UNIT_DIST_M, 1 / smartmouse::maze::UNIT_DIST_M);

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
    painter.setPen(QPen(Qt::black));
    painter.drawLine(line_tf.map(line));
  }
}

void MazeWidget::PaintWalls(QPainter &painter, QTransform tf) {
  for (unsigned int row = 0; row < smartmouse::maze::SIZE; row++) {
    for (unsigned int col= 0; col < smartmouse::maze::SIZE; col++) {
      for (auto wall : maze_walls_[row][col]) {
        double c1 = wall.c1();
        double r1 = wall.r1();
        double c2 = wall.c2();
        double r2 = wall.r2();
        QPainterPath wall_path;
        wall_path.moveTo(c1, r1);
        wall_path.lineTo(c2, r1);
        wall_path.lineTo(c2, r2);
        wall_path.lineTo(c1, r2);
        wall_path.lineTo(c1, r1);
        painter.fillPath(tf.map(wall_path), kWallBrush);
      }
    }
  }
}

const QString MazeWidget::GetTabName() {
  return QString("Maze View");
}

void MazeWidget::OnMaze(const smartmouse::msgs::Maze &msg) {
  smartmouse::msgs::Convert(msg, maze_walls_);
  emit MyUpdate();
}

void MazeWidget::OnRobotDescription(const smartmouse::msgs::RobotDescription &msg) {
  mouse_ = msg;
  mouse_set_ = true;
  emit MyUpdate();
}

void MazeWidget::OnRobotSimState(const smartmouse::msgs::RobotSimState &msg) {
  robot_state_ = msg;

  emit MyUpdate();
}
