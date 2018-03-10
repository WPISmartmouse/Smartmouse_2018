#include <sim/simulator/lib/widgets/ControlPlotWidget.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <msgs/msgs.h>
#include <QtWidgets/QBoxLayout>

#include "ui_controlwidget.h"

ControlPlotWidget::ControlPlotWidget() : ui_(new Ui::ControlPlotWidget()), capacity_(1000) {
  ui_->setupUi(this);

  left_actual_ = new PlotSeriesData("Left Actual", Qt::red, capacity_);
  right_actual_ = new PlotSeriesData("Right Actual", Qt::blue, capacity_);
  ui_->left_checkbox->setStyleSheet("QCheckBox { color: red }");
  ui_->right_checkbox->setStyleSheet("QCheckBox { color: blue }");

  plot_ = new QwtPlot();
  plot_->setMinimumSize(400, 200);

  left_actual_->Attach(plot_);
  right_actual_->Attach(plot_);

  plot_->axisScaleDraw(QwtPlot::xBottom)->enableComponent(QwtAbstractScaleDraw::Labels, false);
  plot_->setAxisTitle(QwtPlot::xBottom, "Time (seconds)");
  plot_->setAxisTitle(QwtPlot::yLeft, "Abstract Force");

  this->node_.Subscribe(TopicNames::kRobotCommand, &ControlPlotWidget::ControlCallback, this);

  ui_->master_layout->addWidget(plot_);

  connect(ui_->clear_button, &QPushButton::clicked, this, &ControlPlotWidget::Clear);
  connect(ui_->left_checkbox, &QCheckBox::stateChanged, this, &ControlPlotWidget::LeftChecked);
  connect(ui_->right_checkbox, &QCheckBox::stateChanged, this, &ControlPlotWidget::RightChecked);
  connect(this, &ControlPlotWidget::Replot, plot_, &QwtPlot::replot, Qt::QueuedConnection);
}

const QString ControlPlotWidget::GetTabName() {
  return QString("Control");
}

void ControlPlotWidget::ControlCallback(const smartmouse::msgs::RobotCommand &msg) {
  double t = smartmouse::msgs::ConvertSec(msg.stamp());

  left_actual_->Append(t, msg.left().abstract_force());
  right_actual_->Append(t, msg.right().abstract_force());

  emit Replot();
}

void ControlPlotWidget::LeftChecked() {
  if (ui_->left_checkbox->isChecked()) {
    left_actual_->Attach(plot_);
  } else {
    left_actual_->Hide();
  }
}

void ControlPlotWidget::RightChecked() {
  if (ui_->right_checkbox->isChecked()) {
    right_actual_->Attach(plot_);
  } else {
    right_actual_->Hide();
  }
}

void ControlPlotWidget::Clear() {
  left_actual_->Clear();
  right_actual_->Clear();
}
