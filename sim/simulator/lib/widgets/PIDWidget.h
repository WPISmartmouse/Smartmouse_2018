#pragma once

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <ignition/transport/Node.hh>
#include <list>

#include <sim/simulator/lib/widgets/AbstractTab.h>
#include <sim/simulator/msgs/pid_debug.pb.h>
#include <qwt_scale_draw.h>
#include <QtWidgets/QPushButton>

class PIDSeriesData : public QwtArraySeriesData<QPointF> {
 public:
  PIDSeriesData(unsigned int capacity);

  virtual QRectF boundingRect() const override;

  void Append(double x, double y);
  void Clear();
  unsigned int capacity_;
  int num_points_to_remove_;
};

class PIDWidget : public QWidget, public AbstractTab {
  Q_OBJECT

 public:
  PIDWidget();

  const QString GetTabName() override;

  void PIDCallback(const smartmouse::msgs::PIDDebug &msg);

  void Clear();

#pragma clang diagnostic push
#pragma ide diagnostic ignored "NotImplementedFunctions"
 signals:
  void Replot();
#pragma clang diagnostic pop

 private:
  ignition::transport::Node node_;
  QPushButton *clear_button_;
  QwtPlot *plot_;
  QwtPlotCurve *left_setpoint_;
  QwtPlotCurve *left_actual_;
  QwtPlotCurve *right_setpoint_;
  QwtPlotCurve *right_actual_;
  PIDSeriesData *left_setpoint_data_;
  PIDSeriesData *left_actual_data_;
  PIDSeriesData *right_setpoint_data_;
  PIDSeriesData *right_actual_data_;
  std::list<std::vector<double>> pid_data_;
  const unsigned  int capacity_;
};
