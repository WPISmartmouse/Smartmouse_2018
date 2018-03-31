#pragma once

#include <ignition/transport/Node.hh>

#include <sim/simulator/lib/widgets/AbstractTab.h>
#include <sim/simulator/msgs/robot_sim_state.pb.h>

namespace Ui {
class SensorWidget;
}

class SensorWidget : public QWidget, public AbstractTab {
  Q_OBJECT

 public:
  SensorWidget();

  const QString GetTabName() override;

  void RoboSimStateCallback(const smartmouse::msgs::RobotSimState &msg);

 private:
  Ui::SensorWidget *ui_;
  ignition::transport::Node node_;
};
