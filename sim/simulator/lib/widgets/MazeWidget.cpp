#include <iostream>

#include <QtGui/QPainter>

#include <common/AbstractMaze.h>
#include <sim/simulator/lib/widgets/MazeWidget.h>
#include <lib/TopicNames.h>

const int MazeWidget::kPaddingPx = 24;
const QBrush MazeWidget::kRobotBrush = QBrush(QColor("#F57C00"));
QBrush MazeWidget::kWallBrush = QBrush(Qt::red);

MazeWidget::MazeWidget() : QWidget() {
  setSizePolicy(QSizePolicy::Policy::MinimumExpanding, QSizePolicy::Policy::MinimumExpanding);
  node_.Subscribe(TopicNames::kMaze, &MazeWidget::OnMaze, this);
  node_.Subscribe(TopicNames::kRobotDescription, &MazeWidget::OnRobotDescription, this);
  node_.Subscribe(TopicNames::kRobotSimState, &MazeWidget::OnRobotSimState, this);
}

/**
 * All drawing should be done in meters, with the origin as the center of the top-left corner.
 * The coordinate system is X increasing across columns, Y increasing across rows, and Z down into the maze.
 * Yes, Z is down so it's a right handed coordinate system. Get over it.
 */
void MazeWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  QTransform tf;
  {

    QRect r = this->geometry();

    int w = std::min(r.width(), r.height()) - kPaddingPx;
    double m2p = w / AbstractMaze::MAZE_SIZE_M;

    int cx = (r.width() - w) / 2;
    int cy = (r.height() - w) / 2;

    tf.translate(cx, cy);
    tf = tf.scale(m2p, m2p);
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
  QPainterPath footprint_;
  footprint_.moveTo(mouse_.footprint(0).x(), mouse_.footprint(0).y());
  for (auto pt : mouse_.footprint()) {
    footprint_.lineTo(pt.x(), pt.y());
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
  QRectF left_wheel_rect(lwx - lr, lwy - lt/2, lr * 2, lt);
  QRectF right_wheel_rect(rwx - rr, rwy - rt/2, rr * 2, rt);

  tf.translate(robot_state_.xytheta().x(), robot_state_.xytheta().y());
  tf.rotateRadians(robot_state_.xytheta().theta(), Qt::ZAxis);

  painter.fillPath(tf.map(footprint_), kRobotBrush);
  painter.fillRect(tf.mapRect(left_wheel_rect), QBrush(Qt::black));
  painter.fillRect(tf.mapRect(right_wheel_rect), QBrush(Qt::black));
}

void MazeWidget::PaintWalls(QPainter &painter, QTransform tf) {
  for (smartmouse::msgs::Wall wall : maze_.walls()) {
    smartmouse::msgs::RowCol row_col = wall.node();
    smartmouse::msgs::Direction::Dir dir = wall.direction();
    double center_row;
    double center_col;
    std::tie(center_row, center_col) = AbstractMaze::rowColToXYCenter(row_col.row(), row_col.col());


    double x1 = center_col, y1 = center_row, w = AbstractMaze::UNIT_DIST, h = AbstractMaze::WALL_THICKNESS;
    switch (dir) {
      case smartmouse::msgs::Direction_Dir_N: {
        x1 = center_col - AbstractMaze::HALF_UNIT_DIST;
        y1 = center_row - AbstractMaze::HALF_UNIT_DIST - AbstractMaze::HALF_WALL_THICKNESS;
        w = AbstractMaze::UNIT_DIST;
        h = AbstractMaze::WALL_THICKNESS;
        break;
      }
      case smartmouse::msgs::Direction_Dir_S: {
        x1 = center_col - AbstractMaze::HALF_UNIT_DIST;
        y1 = center_row + AbstractMaze::HALF_UNIT_DIST - AbstractMaze::HALF_WALL_THICKNESS;
        w = AbstractMaze::UNIT_DIST;
        h = AbstractMaze::WALL_THICKNESS;
        break;
      }
      case smartmouse::msgs::Direction_Dir_E: {
        x1 = center_col + AbstractMaze::HALF_UNIT_DIST - AbstractMaze::HALF_WALL_THICKNESS;
        y1 = center_row - AbstractMaze::HALF_UNIT_DIST;
        w = AbstractMaze::WALL_THICKNESS;
        h = AbstractMaze::UNIT_DIST;
        break;
      }
      case smartmouse::msgs::Direction_Dir_W: {
        x1 = center_col - AbstractMaze::HALF_UNIT_DIST - AbstractMaze::HALF_WALL_THICKNESS;
        y1 = center_row - AbstractMaze::HALF_UNIT_DIST;
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
  emit update();
}

void MazeWidget::OnRobotDescription(const smartmouse::msgs::RobotDescription &msg) {
  mouse_ = msg;
  emit update();
}

void MazeWidget::OnRobotSimState(const smartmouse::msgs::RobotSimState &msg) {
  auto xytheta = robot_state_.mutable_xytheta();
  xytheta->set_x(msg.true_x_meters());
  xytheta->set_y(msg.true_y_meters());
  xytheta->set_theta(msg.true_yaw_rad());

  std::cout << msg.true_x_meters() << std::endl;

  emit update();
}
