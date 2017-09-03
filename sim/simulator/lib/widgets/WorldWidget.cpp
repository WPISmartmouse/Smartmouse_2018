#include <QtGui/QPainter>
#include <common/AbstractMaze.h>
#include "WorldWidget.h"
#include <iostream>

WorldWidget::WorldWidget() {
  setSizePolicy(QSizePolicy::Policy::MinimumExpanding, QSizePolicy::Policy::MinimumExpanding);
}

void WorldWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);

  QRect r = this->geometry();

  int w = std::min(r.width(), r.height()) - 1;
  double m2p = w / AbstractMaze::MAZE_SIZE_M;

  int cx = (r.width() - w) / 2;
  int cy = (r.height() - w) / 2;

  painter.translate(cx, cy);

  QRectF base = QRectF(0, 0, AbstractMaze::MAZE_SIZE_M * m2p, AbstractMaze::MAZE_SIZE_M * m2p);
  painter.drawRect(base);

  for (unsigned int i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    QLineF h_line(0,
                  m2p * (i * AbstractMaze::UNIT_DIST),
                  m2p * AbstractMaze::MAZE_SIZE_M,
                  m2p * (i * AbstractMaze::UNIT_DIST));
    QLineF v_line(m2p * (i * AbstractMaze::UNIT_DIST),
                  0,
                  m2p * (i * AbstractMaze::UNIT_DIST),
                  m2p * AbstractMaze::MAZE_SIZE_M);
    painter.drawLine(h_line);
    painter.drawLine(v_line);
  }
}
