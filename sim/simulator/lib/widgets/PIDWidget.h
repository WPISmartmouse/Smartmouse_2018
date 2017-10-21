#pragma once

#include <QtCharts>
#include <QWidget>
#include <ignition/transport/Node.hh>
#include <list>

#include <sim/simulator/lib/widgets/AbstractTab.h>
#include <sim/simulator/msgs/pid_debug.pb.h>

class PIDWidget : public QChartView, public AbstractTab {
  Q_OBJECT

 public:
  PIDWidget();

  const QString getTabName() override;

  void PIDCallback(const smartmouse::msgs::PIDDebug &msg);

 private:
  ignition::transport::Node node_;
  QChart *chart_;
  QValueAxis *x_axis_;
  QValueAxis *y_axis_;
  QLineSeries *left_setpoint_;
  QLineSeries *left_actual_;
  QLineSeries *right_setpoint_;
  QLineSeries *right_actual_;

  std::list<std::vector<double>> pid_data_;
};

