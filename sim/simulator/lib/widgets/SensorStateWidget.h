#pragma  once

#include <QWidget>

class SensorState : public QWidget {
 Q_OBJECT

 public:
  SensorState();

  double frontLeftWall, frontRightWall, backLeftWall, backRightWall, frontWall;

 protected:
  void paintEvent(QPaintEvent *event);

};
