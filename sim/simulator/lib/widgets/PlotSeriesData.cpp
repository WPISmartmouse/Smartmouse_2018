#include <sim/simulator/lib/widgets/PlotSeriesData.h>
#include <QtWidgets/QBoxLayout>

#include "ui_pidwidget.h"

PlotSeriesData::PlotSeriesData(std::string label, QColor color, const unsigned int capacity)
    : capacity_(capacity), num_points_to_remove_(1) {
  data_mutex_ = new QMutex();
  curve = new QwtPlotCurve(label.c_str());
  curve->setPen(QPen(QBrush(color), 1));
  curve->setData(this);
}

QRectF PlotSeriesData::boundingRect() const {
  return d_boundingRect;
}

void PlotSeriesData::Append(double x, double y) {
  QPointF point(x, y);

  // rate limit the data to once per 1 millisecond
  if (!d_samples.empty() && x - d_samples.back().x() < 0.001) {
    return;
  }

  {
    QMutexLocker lock(data_mutex_);
    d_samples.append(point);

    if (this->d_samples.size() >= (int) capacity_) {
      QPointF removed_pt;
      removed_pt = this->d_samples.takeAt(0);

      // shrink bounding rect
      d_boundingRect.setLeft(removed_pt.x());
    }
  }

  if (this->d_samples.size() == 1) {
    // init bounding rect
    this->d_boundingRect.setTopLeft(point);
    this->d_boundingRect.setBottomRight(point);
    return;
  }

  // expand bounding rect
  if (point.x() < this->d_boundingRect.left())
    this->d_boundingRect.setLeft(point.x());
  else if (point.x() > this->d_boundingRect.right())
    this->d_boundingRect.setRight(point.x());
  if (point.y() < this->d_boundingRect.top())
    this->d_boundingRect.setTop(point.y());
  else if (point.y() > this->d_boundingRect.bottom())
    this->d_boundingRect.setBottom(point.y());
}

void PlotSeriesData::Clear() {
  d_samples.clear();
}

void PlotSeriesData::Hide() {
  curve->detach();
}

void PlotSeriesData::Attach(QwtPlot *plot) {
  curve->attach(plot);
}

size_t PlotSeriesData::size() const {
  QMutexLocker lock(data_mutex_);
  size_t s = d_samples.size();
  return s;
}
