#include <sstream>
#include <cmath>
#include <boost/algorithm/string/replace.hpp>
#include <sim/SimMouse.h>
#include "StateViz.hh"
#include "RegenerateWidget.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(StateViz)

StateViz::StateViz() : GUIPlugin() {

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->state_sub = this->node->Subscribe("~/mouse/state", &StateViz::StateCallback, this);
  this->maze_loc_sub = this->node->Subscribe("~/maze_location", &StateViz::MazeLocationCallback, this);
  this->front_left_analog_sub = this->node->Subscribe("~/mouse/base/front_left_analog/scan", &StateViz::FrontLeftAnalogCallback, this);
  this->front_right_analog_sub = this->node->Subscribe("~/mouse/base/front_right_analog/scan", &StateViz::FrontRightAnalogCallback, this);
  this->back_left_analog_sub = this->node->Subscribe("~/mouse/base/back_left_analog/scan", &StateViz::BackLeftAnalogCallback, this);
  this->back_right_analog_sub = this->node->Subscribe("~/mouse/base/back_right_analog/scan", &StateViz::BackRightAnalogCallback, this);
  this->front_analog_sub = this->node->Subscribe("~/mouse/base/front_analog/scan", &StateViz::FrontAnalogCallback, this);
  this->statsSub = this->node->Subscribe("~/world_stats", &StateViz::OnStats, this);

  this->reset_trace_pub = this->ign_node.Advertise<ignition::msgs::Empty>("/delete_plot");
  if (!reset_trace_pub) {
    gzerr << "Failed to advertise to [" << this->topic << "]" << std::endl;
  }

  this->stop_pub = this->node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");

  QHBoxLayout *main_layout = new QHBoxLayout();

  scroll_area = new QScrollArea(this);
  scroll_area->setLineWidth(1);
  scroll_area->setFrameShape(QFrame::NoFrame);
  scroll_area->setFrameShadow(QFrame::Plain);
  scroll_area->setWidgetResizable(true);
  scroll_area->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
  scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  QFrame *frame = new QFrame(scroll_area);
  frame->setFrameShape(QFrame::NoFrame);

  QVBoxLayout *sensor_layout = new QVBoxLayout();
  QVBoxLayout *info_layout = new QVBoxLayout();
  QHBoxLayout *left_wheel_vel_layout = new QHBoxLayout();
  QHBoxLayout *right_wheel_vel_layout = new QHBoxLayout();
  QHBoxLayout *row_col_layout = new QHBoxLayout();
  QHBoxLayout *dir_layout = new QHBoxLayout();
  QHBoxLayout *x_layout = new QHBoxLayout();
  QHBoxLayout *y_layout = new QHBoxLayout();
  QHBoxLayout *yaw_layout = new QHBoxLayout();
  QHBoxLayout *buttons_layout = new QHBoxLayout();

  scroll_area->setWidget(frame);

  left_wheel_velocity_label = new QLabel(tr("left wheel velocity:"));
  right_wheel_velocity_label = new QLabel(tr("right wheel velocity:"));
  row_label = new QLabel(tr("row:"));
  col_label = new QLabel(tr("column:"));
  true_x_label = new QLabel(tr("true x:"));
  true_y_label = new QLabel(tr("true y:"));
  true_yaw_label = new QLabel(tr("true yaw:"));
  estimated_x_label = new QLabel(tr("estimated x:"));
  estimated_y_label = new QLabel(tr("estimated y:"));
  estimated_yaw_label = new QLabel(tr("estimated yaw:"));

  left_wheel_velocity_edit = new QLineEdit(tr("0.00000 m/sec"));
  right_wheel_velocity_edit = new QLineEdit(tr("0.00000 m/sec"));
  row_edit = new QLineEdit(tr("0"));
  col_edit = new QLineEdit(tr("0"));
  true_x_edit = new QLineEdit(tr("0"));
  true_y_edit = new QLineEdit(tr("0"));
  true_yaw_edit = new QLineEdit(tr("0"));
  estimated_x_edit = new QLineEdit(tr("0"));
  estimated_y_edit = new QLineEdit(tr("0"));
  estimated_yaw_edit = new QLineEdit(tr("0"));

  dir_label = new QLabel(tr("Direction:"));
  dir_edit = new QLineEdit(tr("N"));

  maze_edit = new QTextEdit(tr(""));
  maze_edit->setFont(QFont("Monospace", 6));

  sensor_state = new SensorState();

  QPushButton *clear_plot_button = new QPushButton(tr("Clear Robot Trace"));
  clear_plot_button->setStyleSheet("padding: 1px;");
  connect(clear_plot_button, SIGNAL(clicked()), this, SLOT(ClearRobotTrace()));

  QPushButton *stop_robot_button = new QPushButton(tr("Stop Robot"));
  stop_robot_button->setStyleSheet("padding: 1px;");
  connect(stop_robot_button, SIGNAL(clicked()), this, SLOT(StopRobot()));

  connect(this, SIGNAL(SetLeftVelocity(QString)), this->left_wheel_velocity_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetRightVelocity(QString)), this->right_wheel_velocity_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetRow(QString)), this->row_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetCol(QString)), this->col_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetDir(QString)), this->dir_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueX(QString)), this->true_x_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueY(QString)), this->true_y_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueYaw(QString)), this->true_yaw_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedX(QString)), this->estimated_x_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedY(QString)), this->estimated_y_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedYaw(QString)), this->estimated_yaw_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetMazeEdit(QString)), this->maze_edit,
          SLOT(setPlainText(QString)), Qt::QueuedConnection);

  left_wheel_vel_layout->addWidget(left_wheel_velocity_label);
  left_wheel_vel_layout->addWidget(left_wheel_velocity_edit);
  right_wheel_vel_layout->addWidget(right_wheel_velocity_label);
  right_wheel_vel_layout->addWidget(right_wheel_velocity_edit);
  row_col_layout->addWidget(row_label);
  row_col_layout->addWidget(row_edit);
  row_col_layout->addWidget(col_label);
  row_col_layout->addWidget(col_edit);
  dir_layout->addWidget(dir_label);
  dir_layout->addWidget(dir_edit);
  x_layout->addWidget(true_x_label);
  x_layout->addWidget(true_x_edit);
  x_layout->addWidget(estimated_x_label);
  x_layout->addWidget(estimated_x_edit);
  y_layout->addWidget(true_y_label);
  y_layout->addWidget(true_y_edit);
  y_layout->addWidget(estimated_y_label);
  y_layout->addWidget(estimated_y_edit);
  yaw_layout->addWidget(true_yaw_label);
  yaw_layout->addWidget(true_yaw_edit);
  yaw_layout->addWidget(estimated_yaw_label);
  yaw_layout->addWidget(estimated_yaw_edit);
  buttons_layout->addWidget(clear_plot_button);
  buttons_layout->addWidget(stop_robot_button);
  info_layout->addLayout(left_wheel_vel_layout);
  info_layout->addLayout(right_wheel_vel_layout);
  info_layout->addLayout(row_col_layout);
  info_layout->addLayout(dir_layout);
  info_layout->addLayout(x_layout);
  info_layout->addLayout(y_layout);
  info_layout->addLayout(yaw_layout);
  info_layout->addLayout(buttons_layout);
  sensor_layout->addWidget(sensor_state);
  main_layout->addLayout(sensor_layout);
  main_layout->addLayout(info_layout);
  main_layout->addWidget(maze_edit);
  frame->setLayout(main_layout);
  frame->setContentsMargins(0, 0, 0, 0);

  QPalette pal = palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  this->setAutoFillBackground(true);
  this->setPalette(pal);

  main_layout->setContentsMargins(2, 2, 2, 2);
  this->setLayout(main_layout);

  this->move(RegenerateWidget::WIDTH, 0);
  this->setFixedSize(StateViz::WIDTH, StateViz::HEIGHT);

}

StateViz::~StateViz() {
}

void StateViz::StateCallback(ConstRobotStatePtr &msg) {
  char left_wheel_velocity_str[14];
  snprintf(left_wheel_velocity_str, 14, "%0.2f cm/s", (100 * msg->left_wheel_velocity_mps()));

  char right_wheel_velocity_str[14];
  snprintf(right_wheel_velocity_str, 14, "%0.2f cm/s", (100 * msg->right_wheel_velocity_mps()));

  gazebo::msgs::Pose pose = msg->true_pose();

  this->last_pose = pose;

  char x_str[14];
  snprintf(x_str, 14, "%0.1f cm", msg->true_x_meters() * 100);

  char y_str[14];
  snprintf(y_str, 14, "%0.1f cm", msg->true_y_meters() * 100);

  char yaw_str[15];
  snprintf(yaw_str, 15, "%0.1f deg", (msg->true_yaw_rad() * 180 / M_PI));

  this->SetLeftVelocity(left_wheel_velocity_str);
  this->SetRightVelocity(right_wheel_velocity_str);
  this->SetTrueX(x_str);
  this->SetTrueY(y_str);
  this->SetTrueYaw(yaw_str);
}


void StateViz::MazeLocationCallback(ConstMazeLocationPtr &msg) {
  // compute x and y with respect to the top left square
  char row_str[14];
  snprintf(row_str, 14, "%i (%0.1f cm)", msg->row(), msg->row_offset() * 100);
  char col_str[14];
  snprintf(col_str, 14, "%i (%0.1f cm)", msg->col(), msg->col_offset() * 100);
  this->SetRow(row_str);
  this->SetCol(col_str);
  this->SetDir(QString::fromStdString(msg->dir()));

  char x_str[14];
  snprintf(x_str, 14, "%0.1f cm", msg->estimated_x_meters() * 100);

  char y_str[14];
  snprintf(y_str, 14, "%0.1f cm", msg->estimated_y_meters() * 100);

  char yaw_str[14];
  snprintf(yaw_str, 14, "%0.1f deg", (msg->estimated_yaw_rad() * 180 / M_PI));

  this->SetEstimatedX(x_str);
  this->SetEstimatedY(y_str);
  this->SetEstimatedYaw(yaw_str);
  this->SetMazeEdit(msg->mouse_maze_string().c_str());
}

void StateViz::StopRobot() {
  gazebo::msgs::JointCmd left;
  left.set_name("mouse::left_wheel_joint");
  left.set_force(0);
  stop_pub->Publish(left);

  gazebo::msgs::JointCmd right;
  right.set_name("mouse::right_wheel_joint");
  right.set_force(0);
  stop_pub->Publish(right);

}

void StateViz::ClearRobotTrace() {
  ignition::msgs::Empty msg;
  this->reset_trace_pub.Publish(msg);
}

void StateViz::FrontLeftAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);
  sensor_state->frontLeftWall = raw_range;
}

void StateViz::FrontRightAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);
  sensor_state->frontRightWall = raw_range;
}

void StateViz::BackLeftAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);
  sensor_state->backLeftWall = raw_range;
}

void StateViz::BackRightAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);
  sensor_state->backRightWall = raw_range;
}

void StateViz::FrontAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);
  sensor_state->frontWall = raw_range;
}

SensorState::SensorState() {
  resize(30, StateViz::HEIGHT);
}

QSize SensorState::minimumSizeHint() const {
  return QSize(100, StateViz::HEIGHT);
}

QSize SensorState::sizeHint() const {
  return QSize(100, StateViz::HEIGHT);
}

void SensorState::paintEvent(QPaintEvent * event) {
  QPainter painter(this);

  if (!std::isinf(frontWall) && frontWall < 0.15) {
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(40, 20, 20, 20);
  } else {
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(40, 20, 20, 20);
  }

  if (!std::isinf(frontLeftWall) && frontLeftWall < 0.15) {
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(0, 90, 20, 20);
  } else {
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(0, 90, 20, 20);
  }

  if (!std::isinf(frontRightWall) && frontRightWall < 0.15) {
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(77, 90, 20, 20);
  } else {
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(77, 90, 20, 20);
  }

  if (!std::isinf(backLeftWall) && backLeftWall < 0.15) {
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(0, 145, 20, 20);
  } else {
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(0, 145, 20, 20);
  }

  if (!std::isinf(backRightWall) && backRightWall < 0.15) {
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(77, 145, 20, 20);
  } else {
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(77, 145, 20, 20);
  }

  painter.setPen(QPen(Qt::black));
  char f_str[8];
  snprintf(f_str, 8, "%0.2f cm", frontWall * 100);
  painter.drawText(35, 10, f_str);

  char fl_str[8];
  snprintf(fl_str, 8, "%0.2f cm", frontLeftWall * 100);
  painter.drawText(0, 80, fl_str);

  char fr_str[8];
  snprintf(fr_str, 8, "%0.2f cm", frontLeftWall * 100);
  painter.drawText(55, 80, fl_str);

  char bl_str[8];
  snprintf(bl_str, 8, "%0.2f cm", backLeftWall * 100);
  painter.drawText(0, 140, bl_str);

  char br_str[8];
  snprintf(br_str, 8, "%0.2f cm", backLeftWall * 100);
  painter.drawText(55, 140, br_str);
}

void StateViz::OnStats(ConstWorldStatisticsPtr &msg) {
  sensor_state->update();
}
