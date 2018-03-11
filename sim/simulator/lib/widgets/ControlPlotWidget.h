#pragma once

#include <list>

#include <ignition/transport/Node.hh>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_draw.h>
#include <QtWidgets/QPushButton>

#include <sim/simulator/lib/widgets/AbstractTab.h>
#include <sim/simulator/msgs/robot_command.pb.h>
#include <sim/simulator/lib/widgets/PlotSeriesData.h>

namespace Ui {
class ControlPlotWidget;
}

class ControlPlotWidget : public QWidget, public AbstractTab {
  Q_OBJECT

 public:
  ControlPlotWidget();

  void Clear();
  void Screenshot();

  const QString GetTabName() override;

  void ControlCallback(const smartmouse::msgs::RobotCommand &msg);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "NotImplementedFunctions"
 signals:
  void Replot();
#pragma clang diagnostic pop

 private slots:
  void LeftChecked();
  void RightChecked();

 private:
  Ui::ControlPlotWidget *ui_;
  ignition::transport::Node node_;
  QwtPlot *plot_;
  PlotSeriesData *left_actual_;
  PlotSeriesData *right_actual_;
  const unsigned  int capacity_;
};
