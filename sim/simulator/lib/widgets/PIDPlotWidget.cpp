#include <sim/simulator/lib/widgets/PIDPlotWidget.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <msgs/msgs.h>
#include <QtWidgets/QBoxLayout>

#include "ui_pidwidget.h"

PIDPlotWidget::PIDPlotWidget() : ui_(new Ui::PIDPlotWidget()), capacity_(1000) {
  ui_->setupUi(this);

  left_setpoint_ = new PlotSeriesData("Left Setpoint", Qt::black, capacity_);
  left_actual_ = new PlotSeriesData("Left Actual", Qt::red, capacity_);
  right_setpoint_ = new PlotSeriesData("Right Setpoint", Qt::green, capacity_);
  right_actual_ = new PlotSeriesData("Right Actual", Qt::blue, capacity_);
  ui_->left_setpoint_checkbox->setStyleSheet("QCheckBox { color: black }");
  ui_->left_checkbox->setStyleSheet("QCheckBox { color: red }");
  ui_->right_setpoint_checkbox->setStyleSheet("QCheckBox { color: green }");
  ui_->right_checkbox->setStyleSheet("QCheckBox { color: blue }");

  plot_ = new QwtPlot();
  plot_->setMinimumSize(400, 200);

  left_setpoint_->Attach(plot_);
  left_actual_->Attach(plot_);
  right_setpoint_->Attach(plot_);
  right_actual_->Attach(plot_);

  plot_->setAxisTitle(QwtPlot::xBottom, "Time (seconds)");
  plot_->setAxisTitle(QwtPlot::yLeft, "Speed cell/second");

  this->node_.Subscribe(TopicNames::kDebugState, &PIDPlotWidget::PIDCallback, this);

  ui_->master_layout->addWidget(plot_);

  connect(ui_->clear_button, &QPushButton::clicked, this, &PIDPlotWidget::Clear);
  connect(ui_->left_checkbox, &QCheckBox::stateChanged, this, &PIDPlotWidget::LeftChecked);
  connect(ui_->left_setpoint_checkbox, &QCheckBox::stateChanged, this, &PIDPlotWidget::LeftSetpointChecked);
  connect(ui_->right_checkbox, &QCheckBox::stateChanged, this, &PIDPlotWidget::RightChecked);
  connect(ui_->right_setpoint_checkbox, &QCheckBox::stateChanged, this, &PIDPlotWidget::RightSetpointChecked);
  connect(this, &PIDPlotWidget::Replot, plot_, &QwtPlot::replot, Qt::QueuedConnection);
}

const QString PIDPlotWidget::GetTabName() {
  return QString("PID");
}

void PIDPlotWidget::PIDCallback(const smartmouse::msgs::DebugState &msg) {
  double t = smartmouse::msgs::ConvertSec(msg.stamp());

  left_setpoint_->Append(t, msg.left_cps_setpoint());
  left_actual_->Append(t, msg.left_cps_actual());
  right_setpoint_->Append(t, msg.right_cps_setpoint());
  right_actual_->Append(t, msg.right_cps_actual());

  emit Replot();
}

void PIDPlotWidget::LeftChecked() {
  if (ui_->left_checkbox->isChecked()) {
    left_actual_->Attach(plot_);
  } else {
    left_actual_->Hide();
  }
}

void PIDPlotWidget::LeftSetpointChecked() {
  if (ui_->left_setpoint_checkbox->isChecked()) {
    left_setpoint_->Attach(plot_);
  } else {
    left_setpoint_->Hide();
  }
}

void PIDPlotWidget::RightChecked() {
  if (ui_->right_checkbox->isChecked()) {
    right_actual_->Attach(plot_);
  } else {
    right_actual_->Hide();
  }
}

void PIDPlotWidget::RightSetpointChecked() {
  if (ui_->right_setpoint_checkbox->isChecked()) {
    right_setpoint_->Attach(plot_);
  } else {
    right_setpoint_->Hide();
  }
}

void PIDPlotWidget::Clear() {
  left_setpoint_->Clear();
  left_actual_->Clear();
  right_setpoint_->Clear();
  right_actual_->Clear();
}
