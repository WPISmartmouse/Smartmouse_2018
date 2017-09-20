#include <sim/simulator/lib/widgets/SensorStateWidget.h>
#include <QtGui/QPainter>

SensorState::SensorState() {
}

void SensorState::paintEvent(QPaintEvent * event) {
  QPainter painter(this);

  if (!std::isinf(frontWall) && frontWall < 0.15) {
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(40, 20, 20, 20);
  } else {
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(40, 20, 20, 20);
  }

  if (!std::isinf(frontLeftWall) && frontLeftWall < 0.15) {
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(0, 90, 20, 20);
  } else {
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(0, 90, 20, 20);
  }

  if (!std::isinf(frontRightWall) && frontRightWall < 0.15) {
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(77, 90, 20, 20);
  } else {
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(77, 90, 20, 20);
  }

  if (!std::isinf(backLeftWall) && backLeftWall < 0.15) {
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(0, 145, 20, 20);
  } else {
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(0, 145, 20, 20);
  }

  if (!std::isinf(backRightWall) && backRightWall < 0.15) {
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(77, 145, 20, 20);
  } else {
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(77, 145, 20, 20);
  }

  painter.setPen(QPen(Qt::black));
  char f_str[8];
  snprintf(f_str, 8, "%0.2f cm", frontWall * 100);
  painter.drawText(35, 10, f_str);

  char fl_str[8];
  snprintf(fl_str, 8, "%0.2f cm", frontLeftWall * 100);
  painter.drawText(0, 80, fl_str);

  char fr_str[8];
  snprintf(fr_str, 8, "%0.2f cm", frontLeftWall * 100);
  painter.drawText(55, 80, fl_str);

  char bl_str[8];
  snprintf(bl_str, 8, "%0.2f cm", backLeftWall * 100);
  painter.drawText(0, 140, bl_str);

  char br_str[8];
  snprintf(br_str, 8, "%0.2f cm", backLeftWall * 100);
  painter.drawText(55, 140, br_str);
}

