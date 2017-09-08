#include <iostream>

#include <QtGui/QPainter>

#include <common/AbstractMaze.h>
#include <sim/simulator/lib/widgets/MazeWidget.h>
#include <lib/TopicNames.h>

const int MazeWidget::kPaddingPx = 24;
const QBrush MazeWidget::kRobotBrush = QBrush(QColor("#fe5"));
QBrush MazeWidget::kWallBrush = QBrush(Qt::red);

MazeWidget::MazeWidget() : QWidget() {
  setSizePolicy(QSizePolicy::Policy::MinimumExpanding, QSizePolicy::Policy::MinimumExpanding);
  node_.Subscribe(TopicNames::kMaze, &MazeWidget::OnMaze, this);
  node_.Subscribe(TopicNames::kRobotDescription, &MazeWidget::OnRobotDescription, this);
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
  for (smartmouse::msgs::Wall wall : maze_.walls()) {
    QRectF wall_rect = PaintWall(wall);
    painter.fillRect(tf.mapRect(wall_rect), kWallBrush);
  }

  // Draw the mouse
  QPainterPath footprint_;
  footprint_.moveTo(mouse_.footprint(0).x(), mouse_.footprint(0).y());
  for (auto pt : mouse_.footprint()) {
    footprint_.lineTo(pt.x(), pt.y());
  }

  QTransform mouse_tf;
  mouse_tf.translate(robot_state_.xytheta().x(), robot_state_.xytheta().y());
  mouse_tf.rotateRadians(robot_state_.xytheta().theta(), Qt::ZAxis);
  painter.fillPath(tf.map(footprint_), kRobotBrush);
}

QRectF MazeWidget::PaintWall(smartmouse::msgs::Wall wall) {
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

  return QRectF(x1, y1, w, h);
}

const QString MazeWidget::getTabName() {
  return QString("Maze View");
}

void MazeWidget::OnMaze(const smartmouse::msgs::Maze &msg) {
  maze_ = msg;
  update();
}

void MazeWidget::OnRobotDescription(const smartmouse::msgs::RobotDescription &msg) {
  mouse_ = msg;
  update();
}
