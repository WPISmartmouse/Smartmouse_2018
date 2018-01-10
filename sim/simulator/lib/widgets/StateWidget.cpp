#include <sstream>
#include <QtWidgets/QHBoxLayout>
#include <sim/lib/SimMouse.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <sim/simulator/lib/widgets/StateWidget.h>

#include "ui_statewidget.h"

StateWidget::StateWidget() : AbstractTab(), ui_(new Ui::StateWidget) {
  ui_->setupUi(this);

  pid_widget_ = new PIDPlotWidget();
  wheel_velocity_widget_ = new WheelVelocityPlotWidget();
  ui_->charts_tabs->addTab(wheel_velocity_widget_, wheel_velocity_widget_->GetTabName());
  ui_->charts_tabs->addTab(pid_widget_, pid_widget_->GetTabName());

  this->node_.Subscribe(TopicNames::kRobotSimState, &StateWidget::StateCallback, this);
  this->node_.Subscribe(TopicNames::kRobotCommand, &StateWidget::RobotCommandCallback, this);

  connect(this, SIGNAL(SetLeftVelocity(QString)), ui_->left_velocity_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetRightVelocity(QString)), ui_->right_velocity_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetLeftCurrent(QString)), ui_->left_current_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetRightCurrent(QString)), ui_->right_current_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetLeftAcceleration(QString)), ui_->left_acceleration_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetRightAcceleration(QString)), ui_->right_acceleration_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetLeftForce(QString)), ui_->left_abstract_force_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetRightForce(QString)), ui_->right_abstract_force_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetRow(QString)), ui_->row_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetCol(QString)), ui_->column_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetDir(QString)), ui_->direction_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueCol(QString)), ui_->true_col_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueRow(QString)), ui_->true_row_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueYaw(QString)), ui_->true_yaw_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
}

void StateWidget::StateCallback(const smartmouse::msgs::RobotSimState &msg) {
  this->SetLeftVelocity(QString::asprintf("%0.3f c/s", smartmouse::kc::radToCell(msg.left_wheel().omega())));
  this->SetRightVelocity(QString::asprintf("%0.3f c/s", smartmouse::kc::radToCell(msg.right_wheel().omega())));
  this->SetLeftAcceleration(QString::asprintf("%0.3f c/s^2", smartmouse::kc::radToCell(msg.left_wheel().alpha())));
  this->SetRightAcceleration(QString::asprintf("%0.3f c/s^2", smartmouse::kc::radToCell(msg.right_wheel().alpha())));
  this->SetLeftCurrent(QString::asprintf("%0.3f mA", msg.left_wheel().current() * 1000));
  this->SetRightCurrent(QString::asprintf("%0.3f mA", msg.right_wheel().current() * 1000));
  this->SetTrueCol(QString::asprintf("%0.3f (%0.1f cm)", msg.p().col(), smartmouse::maze::toMeters(msg.p().col()) * 100));
  this->SetTrueRow(QString::asprintf("%0.3f (%0.1f cm)", msg.p().row(), smartmouse::maze::toMeters(msg.p().col()) * 100));
  this->SetTrueYaw(QString::asprintf("%0.1f deg", msg.p().theta() * 180 / M_PI));

  // compute x and y with respect to the top left square
  char col_str[14];
  snprintf(col_str, 14, "%i", (int) msg.p().col());
  char row_str[14];
  snprintf(row_str, 14, "%i", (int) msg.p().row());
  this->SetCol(col_str);
  this->SetRow(row_str);
  this->SetDir(QChar(yaw_to_char(msg.p().theta())));
}

void StateWidget::RobotCommandCallback(const smartmouse::msgs::RobotCommand &msg) {
  this->SetLeftForce(QString::asprintf("%3i / 255", msg.left().abstract_force()));
  this->SetRightForce(QString::asprintf("%3i / 255", msg.right().abstract_force()));
}

const QString StateWidget::GetTabName() {
  return QString("State Widget");
}
