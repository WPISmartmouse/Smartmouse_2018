#include <QtGui/QPainter>
#include <iostream>
#include "WorldWidget.h"

WorldWidget::WorldWidget() {
  setFixedSize(400, 400);
  std::cout << "hello" << std::endl;
}

void WorldWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);

  painter.setBrush(QBrush(Qt::green));
  painter.drawRect(0, 0, 100, 100);
}
