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

  tf.translate(robot_state_.xytheta().x(), robot_state_.xytheta().y());
  tf.rotateRadians(robot_state_.xytheta().theta(), Qt::ZAxis);

  painter.setPen(QPen(Qt::black));
  painter.fillPath(tf.map(footprint), kRobotBrush);
  painter.fillPath(tf.map(left_wheel_path), QBrush(Qt::black));
  painter.fillPath(tf.map(right_wheel_path), QBrush(Qt::black));

  for (auto sensors : mouse_.sensors()) {
    QTransform line_tf(tf);
    line_tf.translate(sensors.x(), sensors.y());
    line_tf.rotateRadians(sensors.yaw(), Qt::ZAxis);
    QLineF line(0, 0, 0.1, 0);
    painter.setPen(QPen(Qt::green));
    painter.drawLine(line_tf.map(line));
  }
}

void MazeWidget::PaintWalls(QPainter &painter, QTransform tf) {
  for (smartmouse::msgs::Wall wall : maze_.walls()) {
    smartmouse::msgs::RowCol row_col = wall.node();
    smartmouse::msgs::Direction::Dir dir = wall.direction();
    double center_x, center_y;
    std::tie(center_x, center_y) = AbstractMaze::rowColToXYCenter(row_col.row(), row_col.col());

    double x1 = center_x, y1 = center_y, w = AbstractMaze::UNIT_DIST, h = AbstractMaze::WALL_THICKNESS;
    switch (dir) {
      case smartmouse::msgs::Direction_Dir_N: {
        x1 = center_x - AbstractMaze::HALF_UNIT_DIST;
        y1 = center_y + AbstractMaze::HALF_UNIT_DIST - AbstractMaze::HALF_WALL_THICKNESS;
        w = AbstractMaze::UNIT_DIST;
        h = AbstractMaze::WALL_THICKNESS;
        break;
      }
      case smartmouse::msgs::Direction_Dir_S: {
        x1 = center_x - AbstractMaze::HALF_UNIT_DIST;
        y1 = center_y - AbstractMaze::HALF_UNIT_DIST - AbstractMaze::HALF_WALL_THICKNESS;
        w = AbstractMaze::UNIT_DIST;
        h = AbstractMaze::WALL_THICKNESS;
        break;
      }
      case smartmouse::msgs::Direction_Dir_E: {
        x1 = center_x + AbstractMaze::HALF_UNIT_DIST - AbstractMaze::HALF_WALL_THICKNESS;
        y1 = center_y - AbstractMaze::HALF_UNIT_DIST;
        w = AbstractMaze::WALL_THICKNESS;
        h = AbstractMaze::UNIT_DIST;
        break;
      }
      case smartmouse::msgs::Direction_Dir_W: {
        x1 = center_x - AbstractMaze::HALF_UNIT_DIST - AbstractMaze::HALF_WALL_THICKNESS;
        y1 = center_y - AbstractMaze::HALF_UNIT_DIST;
        w = AbstractMaze::WALL_THICKNESS;
        h = AbstractMaze::UNIT_DIST;
        break;
      }
    }

    painter.fillRect(tf.mapRect(QRectF(x1, y1, w, h)), kWallBrush);
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
  auto xytheta = robot_state_.mutable_xytheta();
  xytheta->set_x(msg.p().x());
  xytheta->set_y(msg.p().y());
  xytheta->set_theta(msg.p().theta());

  emit MyUpdate();
}
