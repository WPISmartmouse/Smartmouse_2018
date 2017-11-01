#include <QtWidgets/QBoxLayout>

#include <common/KinematicController/KinematicController.h>
#include <sim/simulator/lib/widgets/WheelVelocityPlotWidget.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <sim/simulator/msgs/msgs.h>

#include "ui_wheelvelocitywidget.h"

WheelVelocityPlotWidget::WheelVelocityPlotWidget() : ui_(new Ui::WheelVelocityPlotWidget()), capacity_(2000) {
  ui_->setupUi(this);

  left_velocity_ = new PlotSeriesData("Left", Qt::red, capacity_);
  right_velocity_ = new PlotSeriesData("Right", Qt::blue, capacity_);

  plot_ = new QwtPlot();
  plot_->setMinimumSize(400, 200);

  left_velocity_->Attach(plot_);
  right_velocity_->Attach(plot_);

  plot_->axisScaleDraw(QwtPlot::xBottom)->enableComponent(QwtAbstractScaleDraw::Labels, false);
  plot_->setAxisTitle(QwtPlot::xBottom, "Time (seconds)");
  plot_->setAxisTitle(QwtPlot::yLeft, "Speed cells/second");

  this->node_.Subscribe(TopicNames::kRobotSimState, &WheelVelocityPlotWidget::WheelVelocityCallback, this);

  ui_->master_layout->addWidget(plot_);

  connect(ui_->clear_button, &QPushButton::clicked, this, &WheelVelocityPlotWidget::Clear);
  connect(ui_->left_checkbox, &QCheckBox::stateChanged, this, &WheelVelocityPlotWidget::LeftChecked);
  connect(ui_->right_checkbox, &QCheckBox::stateChanged, this, &WheelVelocityPlotWidget::RightChecked);
  connect(this, &WheelVelocityPlotWidget::Replot, plot_, &QwtPlot::replot, Qt::QueuedConnection);
}

const QString WheelVelocityPlotWidget::GetTabName() {
  return QString("Wheel Velocities");
}

void WheelVelocityPlotWidget::WheelVelocityCallback(const smartmouse::msgs::RobotSimState &msg) {
  double t = smartmouse::msgs::ConvertSec(msg.stamp());

  left_velocity_->Append(t, smartmouse::kc::radToCell(msg.left_wheel().omega()));
  right_velocity_->Append(t, smartmouse::kc::radToCell(msg.right_wheel().omega()));

  emit Replot();
}

void WheelVelocityPlotWidget::LeftChecked() {
  if (ui_->left_checkbox->isChecked()) {
    left_velocity_->Attach(plot_);
  } else {
    left_velocity_->Hide();
  }
}

void WheelVelocityPlotWidget::RightChecked() {
  if (ui_->right_checkbox->isChecked()) {
    right_velocity_->Attach(plot_);
  } else {
    right_velocity_->Hide();
  }
}

void WheelVelocityPlotWidget::Clear() {
  left_velocity_->Clear();
  right_velocity_->Clear();
}
