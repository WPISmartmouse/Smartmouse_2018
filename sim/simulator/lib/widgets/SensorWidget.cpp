#include <sim/simulator/lib/widgets/SensorWidget.h>
#include <sim/simulator/lib/common/TopicNames.h>

#include "ui_sensorwidget.h"

SensorWidget::SensorWidget() : AbstractTab(), ui_(new Ui::SensorWidget) {
  ui_->setupUi(this);
  this->node_.Subscribe(TopicNames::kRobotSimState, &SensorWidget::RoboSimStateCallback, this);
}

const QString SensorWidget::GetTabName() {
  return QString("Range Sensors");
}

void SensorWidget::RoboSimStateCallback(const smartmouse::msgs::RobotSimState &msg) {
  ui_->back_left_label->setText(QString::number(msg.back_left()));
  ui_->front_left_label->setText(QString::number(msg.front_left()));
  ui_->gerald_left_label->setText(QString::number(msg.gerald_left()));
  ui_->front_label->setText(QString::number(msg.front()));
  ui_->back_right_label->setText(QString::number(msg.back_right()));
  ui_->front_right_label->setText(QString::number(msg.front_right()));
  ui_->gerald_right_label->setText(QString::number(msg.gerald_right()));
}
