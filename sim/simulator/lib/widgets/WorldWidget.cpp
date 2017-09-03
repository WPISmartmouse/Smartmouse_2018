#include <QtGui/QPainter>
#include "WorldWidget.h"

WorldWidget::WorldWidget() {
  setSizePolicy(QSizePolicy::Policy::MinimumExpanding, QSizePolicy::Policy::MinimumExpanding);
}

void WorldWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);

  painter.setBrush(QBrush(Qt::green));
  QRect r = this->geometry();
  int w = std::min(r.width(), r.height()) - 1;
  QRect base = QRect(0, 0, w, w);
  int cx = (r.width() - w) / 2;
  int cy = (r.height() - w) / 2;
  base.translate(cx, cy);
  painter.drawRect(base);
}
