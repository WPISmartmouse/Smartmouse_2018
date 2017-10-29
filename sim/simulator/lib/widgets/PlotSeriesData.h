#pragma once

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <ignition/transport/Node.hh>
#include <list>

#include <sim/simulator/lib/widgets/AbstractTab.h>
#include <sim/simulator/msgs/pid_debug.pb.h>
#include <qwt_scale_draw.h>
#include <QtWidgets/QPushButton>

namespace Ui {
class PlotSeriesData;
}

class PlotWidget;

class PlotSeriesData : public QwtArraySeriesData<QPointF> {

 public:
  PlotSeriesData(std::string label, QColor color=Qt::black, const unsigned int capacity=1000);

  virtual QRectF boundingRect() const override;

  void Append(double x, double y);
  void Clear();
  void Hide();
  void Attach(QwtPlot *plot_);

 private:
  unsigned int capacity_;
  int num_points_to_remove_;
  QwtPlotCurve *curve;
};
