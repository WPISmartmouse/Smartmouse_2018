#include <sim/simulator/lib/widgets/PIDWidget.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <msgs/msgs.h>

PIDWidget::PIDWidget() : capacity_(100) {
  left_setpoint_ = new QwtPlotCurve("left setpoint");
  left_actual_ = new QwtPlotCurve("left actual");
  right_setpoint_ = new QwtPlotCurve("right setpoint");
  right_actual_ = new QwtPlotCurve("right actual");

  left_setpoint_data_ = new PIDSeriesData(capacity_);
  left_actual_data_ = new PIDSeriesData(capacity_);
  right_setpoint_data_ = new PIDSeriesData(capacity_);
  right_actual_data_ = new PIDSeriesData(capacity_);

  left_setpoint_->setData(left_setpoint_data_);
  left_actual_->setData(left_actual_data_);
  right_setpoint_->setData(right_setpoint_data_);
  right_actual_->setData(right_actual_data_);

  left_setpoint_->attach(this);
  left_actual_->attach(this);
  right_setpoint_->attach(this);
  right_actual_->attach(this);
  left_setpoint_->setPen(QPen(Qt::blue));
  left_actual_->setPen(QPen(Qt::red));
  right_setpoint_->setPen(QPen(Qt::green));
  right_actual_->setPen(QPen(Qt::yellow));

  this->axisScaleDraw(QwtPlot::xBottom)->enableComponent(QwtAbstractScaleDraw::Labels, false);
  this->setAxisTitle(QwtPlot::xBottom, "Time (seconds)");
  this->setAxisTitle(QwtPlot::yLeft, "Speed ms<sup>-1</sup>");

  this->node_.Subscribe(TopicNames::kPID, &PIDWidget::PIDCallback, this);

  connect(this, &PIDWidget::Replot, this, &PIDWidget::replot);
}

const QString PIDWidget::getTabName() {
  return QString("PID");
}

void PIDWidget::PIDCallback(const smartmouse::msgs::PIDDebug &msg) {
  static int i = 0;
  double t = smartmouse::msgs::ConvertSec(msg.stamp());
  double x = ((float) rand()) / RAND_MAX;
  double y = 0.25 * ((float) rand()) / RAND_MAX;
  printf("%f, %f\n", t, msg.left_mps_actual());
  right_setpoint_data_->append(t, msg.left_mps_setpoint());
  left_setpoint_data_->append(t, msg.left_mps_setpoint());
  left_actual_data_->append(t, msg.left_mps_actual());
  right_setpoint_data_->append(t, msg.right_mps_setpoint());
  right_actual_data_->append(t, msg.right_mps_actual());

  emit Replot();
}

PIDSeriesData::PIDSeriesData(unsigned int capacity) : capacity_(capacity), num_points_to_remove_(1) {}

QRectF PIDSeriesData::boundingRect() const {
//  std::cout << d_boundingRect.left() << " " << d_boundingRect.top() << " " << d_boundingRect.right() << " "
//            << d_boundingRect.bottom() << std::endl;
  return d_boundingRect;
}

void PIDSeriesData::append(double x, double y) {
  QPointF point(x, y);
  d_samples.append(point);

  if (this->d_samples.size() > capacity_) {
    QPointF removed_pt = this->d_samples.takeAt(0);

    // shrink bounding rect
     d_boundingRect.setLeft(removed_pt.x());
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
