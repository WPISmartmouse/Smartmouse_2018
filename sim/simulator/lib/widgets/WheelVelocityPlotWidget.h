#pragma once

#include <list>

#include <ignition/transport/Node.hh>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_draw.h>
#include <QtWidgets/QPushButton>

#include <sim/simulator/lib/widgets/AbstractTab.h>
#include <sim/simulator/lib/widgets/PlotSeriesData.h>
#include <msgs/robot_sim_state.pb.h>

namespace Ui {
class WheelVelocityPlotWidget;
}

class WheelVelocityPlotWidget : public QWidget, public AbstractTab {
  Q_OBJECT

 public:
  WheelVelocityPlotWidget();

  void Clear();

  const QString GetTabName() override;

  void WheelVelocityCallback(const smartmouse::msgs::RobotSimState &msg);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "NotImplementedFunctions"
 signals:
  void Replot();
#pragma clang diagnostic pop

 private slots:
  void LeftChecked();
  void RightChecked();

 private:
  Ui::WheelVelocityPlotWidget *ui_;
  ignition::transport::Node node_;
  QwtPlot *plot_;
  PlotSeriesData *left_velocity_;
  PlotSeriesData *right_velocity_;
  const unsigned  int capacity_;
};
