#pragma once

#include <list>

#include <ignition/transport/Node.hh>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_draw.h>
#include <QtWidgets/QPushButton>

#include <sim/simulator/lib/widgets/AbstractTab.h>
#include <sim/simulator/msgs/debug_state.pb.h>
#include <sim/simulator/lib/widgets/PlotSeriesData.h>

namespace Ui {
class PIDPlotWidget;
}

class PIDPlotWidget : public QWidget, public AbstractTab {
  Q_OBJECT

 public:
  PIDPlotWidget();

  void Clear();
  void Screenshot();

  const QString GetTabName() override;

  void PIDCallback(const smartmouse::msgs::DebugState &msg);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "NotImplementedFunctions"
 signals:
  void Replot();
#pragma clang diagnostic pop

 private slots:
  void LeftChecked();
  void LeftSetpointChecked();
  void RightChecked();
  void RightSetpointChecked();

 private:
  Ui::PIDPlotWidget *ui_;
  ignition::transport::Node node_;
  QwtPlot *plot_;
  PlotSeriesData *left_setpoint_;
  PlotSeriesData *left_actual_;
  PlotSeriesData *right_setpoint_;
  PlotSeriesData *right_actual_;
  const unsigned  int capacity_;
};
