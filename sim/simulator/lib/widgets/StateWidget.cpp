#include <sstream>
#include <cmath>
#include <QtWidgets/QHBoxLayout>
#include <common/math/math.h>
#include <sim/lib/SimMouse.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <sim/simulator/lib/widgets/StateWidget.h>

#include "ui_statewidget.h"

StateWidget::StateWidget() : AbstractTab(), ui_(new Ui::StateWidget) {
  ui_->setupUi(this);

  this->node.Subscribe(TopicNames::kRobotSimState, &StateWidget::StateCallback, this);
  this->node.Subscribe(TopicNames::kRobotCommand, &StateWidget::RobotCommandCallback, this);
  this->node.Subscribe(TopicNames::kMazeLocation, &StateWidget::MazeLocationCallback, this);
  this->node.Subscribe(TopicNames::kWorldStatistics, &StateWidget::OnStats, this);

  sensor_state = new SensorState();

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
  connect(this, SIGNAL(SetTrueX(QString)), ui_->true_x_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueY(QString)), ui_->true_y_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueYaw(QString)), ui_->true_yaw_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedX(QString)), ui_->estimated_x_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedY(QString)), ui_->estimated_y_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedYaw(QString)), ui_->estimated_yaw_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(HighlightX(QString)), ui_->estimated_x_edit,
          SLOT(setStyleSheet(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(HighlightY(QString)), ui_->estimated_y_edit,
          SLOT(setStyleSheet(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(HighlightYaw(QString)), ui_->estimated_yaw_edit,
          SLOT(setStyleSheet(QString)), Qt::QueuedConnection);
}

void StateWidget::StateCallback(const smartmouse::msgs::RobotSimState &msg) {
  double left_wheel_velocity_mps = smartmouse::kc::radToMeters(100 * msg.left_wheel().omega());
  double right_wheel_velocity_mps = smartmouse::kc::radToMeters(100 * msg.right_wheel().omega());
  double left_wheel_acceleration_mpss = smartmouse::kc::radToMeters(100 * msg.left_wheel().alpha());
  double right_wheel_acceleration_mpss = smartmouse::kc::radToMeters(100 * msg.right_wheel().alpha());

  this->true_x = smartmouse::maze::toMeters(msg.p().row());
  this->true_y = smartmouse::maze::toMeters(msg.p().col());
  this->true_yaw = msg.p().theta();

  this->SetLeftVelocity(QString::asprintf("%0.3f cm/s", left_wheel_velocity_mps));
  this->SetRightVelocity(QString::asprintf("%0.3f cm/s", right_wheel_velocity_mps));
  this->SetLeftAcceleration(QString::asprintf("%0.3f cm/s^2", left_wheel_acceleration_mpss));
  this->SetRightAcceleration(QString::asprintf("%0.3f cm/s^2", right_wheel_acceleration_mpss));
  this->SetLeftCurrent(QString::asprintf("%0.3f mA", msg.left_wheel().current() * 1000));
  this->SetRightCurrent(QString::asprintf("%0.3f mA", msg.right_wheel().current() * 1000));
  this->SetTrueX(QString::asprintf("%0.1f cm", true_x * 100));
  this->SetTrueY(QString::asprintf("%0.1f cm", true_y * 100));
  this->SetTrueYaw(QString::asprintf("%0.1f deg", true_yaw * 180 / M_PI));
}

void StateWidget::MazeLocationCallback(const smartmouse::msgs::MazeLocation &msg) {
  // compute x and y with respect to the top left square
  char row_str[14];
  snprintf(row_str, 14, "%i (%0.1f cm)", msg.row(), msg.row_offset() * 100);
  char col_str[14];
  snprintf(col_str, 14, "%i (%0.1f cm)", msg.col(), msg.col_offset() * 100);
  this->SetRow(row_str);
  this->SetCol(col_str);
  this->SetDir(QString::fromStdString(msg.dir()));

  char x_str[14];
  snprintf(x_str, 14, "%0.1f cm", msg.estimated_x_meters() * 100);

  char y_str[14];
  snprintf(y_str, 14, "%0.1f cm", msg.estimated_y_meters() * 100);

  char yaw_str[14];
  snprintf(yaw_str, 14, "%0.1f deg", (msg.estimated_yaw_rad() * 180 / M_PI));

  if (fabs(msg.estimated_x_meters() - true_x) > 0.01) {
    this->HighlightX("QLineEdit {color:red;}");
  } else {
    this->HighlightX("QLineEdit {color:black;}");
  }
  if (fabs(msg.estimated_y_meters() - true_y) > 0.01) {
    this->HighlightY("QLineEdit {color:red;}");
  } else {
    this->HighlightY("QLineEdit {color:black;}");
  }
  if (smartmouse::math::yawDiff(msg.estimated_yaw_rad(), true_yaw) > 0.02) {
    this->HighlightYaw("QLineEdit {color:red;}");
  } else {
    this->HighlightYaw("QLineEdit {color:black;}");
  }

  this->SetEstimatedX(x_str);
  this->SetEstimatedY(y_str);
  this->SetEstimatedYaw(yaw_str);
}

void StateWidget::OnStats(const smartmouse::msgs::WorldStatistics &msg) {
  sensor_state->update();
}

void StateWidget::RobotCommandCallback(const smartmouse::msgs::RobotCommand &msg) {
  this->SetLeftForce(QString::asprintf("%3i /255", msg.left().abstract_force()));
  this->SetRightForce(QString::asprintf("%3i /255", msg.right().abstract_force()));
}

const QString StateWidget::getTabName() {
  return QString("State Widget");
}
