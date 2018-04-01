#pragma once

#include <ignition/transport/Node.hh>

#include <sim/simulator/lib/widgets/AbstractTab.h>
#include <sim/simulator/msgs/robot_sim_state.pb.h>
#include <sim/simulator/msgs/debug_state.pb.h>

namespace Ui {
class SensorWidget;
}

class SensorWidget : public QWidget, public AbstractTab {
  Q_OBJECT

 public:
  SensorWidget();

  const QString GetTabName() override;

  void RobotSimStateCallback(const smartmouse::msgs::RobotSimState &msg);

  void DebugStateCallback(const smartmouse::msgs::DebugState &msg);

 signals:

#pragma clang diagnostic push
#pragma ide diagnostic ignored "NotImplementedFunctions"
  void SetTrueBackLeft(QString str);
  void SetTrueFrontLeft(QString str);
  void SetTrueGeraldLeft(QString str);
  void SetTrueFront(QString str);
  void SetTrueGeraldRight(QString str);
  void SetTrueFrontRight(QString str);
  void SetTrueBackRight(QString str);
  void SetEstimatedBackLeft(QString str);
  void SetEstimatedFrontLeft(QString str);
  void SetEstimatedGeraldLeft(QString str);
  void SetEstimatedFront(QString str);
  void SetEstimatedGeraldRight(QString str);
  void SetEstimatedFrontRight(QString str);
  void SetEstimatedBackRight(QString str);
#pragma clang diagnostic pop

 private:
  Ui::SensorWidget *ui_;
  ignition::transport::Node node_;
};
