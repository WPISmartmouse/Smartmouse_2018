#include <iostream>

#include <QtGui/QPainter>

#include <common/AbstractMaze.h>
#include <sim/simulator/lib/widgets/MazeWidget.h>

const int MazeWidget::PADDING_PX = 24;

MazeWidget::MazeWidget() : QWidget() {
  setSizePolicy(QSizePolicy::Policy::MinimumExpanding, QSizePolicy::Policy::MinimumExpanding);
}

void MazeWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  QTransform tf;
  {

    QRect r = this->geometry();

    int w = std::min(r.width(), r.height()) - PADDING_PX;
    double m2p = w / AbstractMaze::MAZE_SIZE_M;

    int cx = (r.width() - w) / 2;
    int cy = (r.height() - w) / 2;

    tf.translate(cx, cy);
    tf = tf.scale(m2p, m2p);
  }

  QRectF base = QRectF(0, 0, AbstractMaze::MAZE_SIZE_M, AbstractMaze::MAZE_SIZE_M);
  painter.drawRect(tf.mapRect(base));

  for (unsigned int i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    QLineF h_line(0, i * AbstractMaze::UNIT_DIST, AbstractMaze::MAZE_SIZE_M, i * AbstractMaze::UNIT_DIST);
    painter.drawLine(tf.map(h_line));

    QLineF v_line((i * AbstractMaze::UNIT_DIST), 0, (i * AbstractMaze::UNIT_DIST), AbstractMaze::MAZE_SIZE_M);
    painter.drawLine(tf.map(v_line));
  }
}
const QString MazeWidget::getTabName() {
  return QString("Maze View");
}
