#pragma once

#include <ignition/transport/Node.hh>
#include <QtGui/QPainter>
#include <QLineEdit>
#include <QTextEdit>
#include <QScrollArea>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtWidgets/QLabel>

#include <sim/simulator/lib/widgets/AbstractTab.h>
#include <sim/simulator/lib/widgets/PIDPlotWidget.h>
#include <sim/simulator/lib/widgets/ControlPlotWidget.h>
#include <sim/simulator/msgs/debug_state.pb.h>
#include <sim/simulator/msgs/robot_command.pb.h>
#include <sim/simulator/msgs/robot_sim_state.pb.h>
#include <sim/simulator/msgs/world_statistics.pb.h>

namespace Ui {
class StateWidget;
}

class StateWidget : public QWidget, public AbstractTab {
 Q_OBJECT

 public:
  StateWidget();

  const QString GetTabName() override;

 signals:

#pragma clang diagnostic push
#pragma ide diagnostic ignored "NotImplementedFunctions"
  void SetLeftVelocity(QString str);
  void SetRightVelocity(QString str);
  void SetLeftCurrent(QString str);
  void SetRightCurrent(QString str);
  void SetLeftAcceleration(QString str);
  void SetRightAcceleration(QString str);
  void SetLeftForce(QString str);
  void SetRightForce(QString str);
  void SetRow(QString str);
  void SetCol(QString str);
  void SetDir(QString str);
  void SetTrueCol(QString str);
  void SetTrueRow(QString str);
  void SetTrueYaw(QString str);
  void SetEstimatedCol(QString str);
  void SetEstimatedRow(QString str);
  void SetEstimatedYaw(QString str);
  void HighlightCol(QString str);
  void HighlightRow(QString str);
  void HighlightYaw(QString str);
#pragma clang diagnostic pop

 private:

  void RobotCommandCallback(const smartmouse::msgs::RobotCommand &msg);
  void DebugStateCallback(const smartmouse::msgs::DebugState &msg);
  void StateCallback(const smartmouse::msgs::RobotSimState &msg);

  double true_col, true_row, true_yaw;

  ignition::transport::Node node_;

  Ui::StateWidget *ui_;
  PIDPlotWidget *pid_widget_;
  ControlPlotWidget *control_widget_;
};
