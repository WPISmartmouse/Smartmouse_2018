#include <sim/simulator/lib/widgets/PIDWidget.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <msgs/msgs.h>
#include <QtWidgets/QBoxLayout>

#include "ui_pidwidget.h"

PIDWidget::PIDWidget() : ui_(new Ui::PIDWidget()), capacity_(1000) {
  ui_->setupUi(this);

  left_setpoint_ = new QwtPlotCurve("left setpoint");
  left_actual_ = new QwtPlotCurve("left actual");
  right_setpoint_ = new QwtPlotCurve("right setpoint");
  right_actual_ = new QwtPlotCurve("right actual");

  left_setpoint_data_ = new PIDSeriesData(capacity_);
  left_actual_data_ = new PIDSeriesData(capacity_);
  right_setpoint_data_ = new PIDSeriesData(capacity_);
  right_actual_data_ = new PIDSeriesData(capacity_);

  plot_ = new QwtPlot();

  left_setpoint_->setData(left_setpoint_data_);
  left_actual_->setData(left_actual_data_);
  right_setpoint_->setData(right_setpoint_data_);
  right_actual_->setData(right_actual_data_);

  // order here matters. It determines drawing order
  left_setpoint_->attach(plot_);
  right_setpoint_->attach(plot_);
  left_actual_->attach(plot_);
  right_actual_->attach(plot_);

  left_setpoint_->setPen(QPen(QBrush(Qt::black), 1));
  left_actual_->setPen(QPen(QBrush(Qt::blue), 1));
  right_setpoint_->setPen(QPen(QBrush(Qt::red), 1));
  right_actual_->setPen(QPen(QBrush(Qt::green), 1));

  plot_->axisScaleDraw(QwtPlot::xBottom)->enableComponent(QwtAbstractScaleDraw::Labels, false);
  plot_->setAxisTitle(QwtPlot::xBottom, "Time (seconds)");
  plot_->setAxisTitle(QwtPlot::yLeft, "Speed ms<sup>-1</sup>");

  this->node_.Subscribe(TopicNames::kPID, &PIDWidget::PIDCallback, this);

  ui_->master_layout->addWidget(plot_);

  connect(ui_->clear_button, &QPushButton::clicked, this, &PIDWidget::Clear);
  connect(ui_->left_checkbox, &QCheckBox::stateChanged, this, &PIDWidget::LeftChecked);
  connect(ui_->left_setpoint_checkbox, &QCheckBox::stateChanged, this, &PIDWidget::LeftSetpointChecked);
  connect(ui_->right_checkbox, &QCheckBox::stateChanged, this, &PIDWidget::RightChecked);
  connect(ui_->right_setpoint_checkbox, &QCheckBox::stateChanged, this, &PIDWidget::RightSetpointChecked);
  connect(this, &PIDWidget::Replot, plot_, &QwtPlot::replot, Qt::QueuedConnection);
}

const QString PIDWidget::GetTabName() {
  return QString("PID");
}

void PIDWidget::PIDCallback(const smartmouse::msgs::PIDDebug &msg) {
  double t = smartmouse::msgs::ConvertSec(msg.stamp());

  left_setpoint_data_->Append(t, msg.left_mps_setpoint());
  left_actual_data_->Append(t, msg.left_mps_actual());
  right_setpoint_data_->Append(t, msg.right_mps_setpoint());
  right_actual_data_->Append(t, msg.right_mps_actual());

  emit Replot();
}

void PIDWidget::Clear() {
  left_setpoint_data_->Clear();
  left_actual_data_->Clear();
  right_setpoint_data_->Clear();
  right_actual_data_->Clear();
  emit Replot();
}

void PIDWidget::LeftChecked() {
  if (ui_->left_checkbox->isChecked()) {
    left_actual_->attach(plot_);
  } else {
    left_actual_->detach();
  }
}

void PIDWidget::LeftSetpointChecked() {
  if (ui_->left_setpoint_checkbox->isChecked()) {
    left_setpoint_->attach(plot_);
  } else {
    left_setpoint_->detach();
  }
}

void PIDWidget::RightChecked() {
  if (ui_->right_checkbox->isChecked()) {
    right_actual_->attach(plot_);
  } else {
    right_actual_->detach();
  }
}

void PIDWidget::RightSetpointChecked() {
  if (ui_->right_setpoint_checkbox->isChecked()) {
    right_setpoint_->attach(plot_);
  } else {
    right_setpoint_->detach();
  }
}

PIDSeriesData::PIDSeriesData(unsigned int capacity) : capacity_(capacity), num_points_to_remove_(1) {}

QRectF PIDSeriesData::boundingRect() const {
  return d_boundingRect;
}

void PIDSeriesData::Append(double x, double y) {
  QPointF point(x, y);
  d_samples.append(point);

  if (this->d_samples.size() > (int) capacity_) {
    QPointF removed_pt;
    removed_pt = this->d_samples.takeAt(0);

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

void PIDSeriesData::Clear() {
  d_samples.clear();
}
