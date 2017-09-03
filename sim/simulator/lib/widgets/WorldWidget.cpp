#include <QtGui/QPainter>
#include <common/AbstractMaze.h>
#include "WorldWidget.h"
#include <iostream>

WorldWidget::WorldWidget() {
  setSizePolicy(QSizePolicy::Policy::MinimumExpanding, QSizePolicy::Policy::MinimumExpanding);
}

void WorldWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  QTransform tf;
  {

    QRect r = this->geometry();

    int w = std::min(r.width(), r.height()) - 1;
    double m2p = w / AbstractMaze::MAZE_SIZE_M;

    int cx = (r.width() - w) / 2;
    int cy = (r.height() - w) / 2;
    tf.translate(cx, cy);
    tf = QTransform::fromScale(m2p, m2p);
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
