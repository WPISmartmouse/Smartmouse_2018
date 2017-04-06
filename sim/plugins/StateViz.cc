#include <sstream>
#include <cmath>
#include <boost/algorithm/string/replace.hpp>
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

  this->reset_trace_pub = this->ign_node.Advertise<ignition::msgs::Empty>("/delete_plot");
  if (!reset_trace_pub) {
    gzerr << "Failed to advertise to [" << this->topic << "]" << std::endl;
  }

  this->stop_pub = this->node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");

  QHBoxLayout *main_layout = new QHBoxLayout();
  QVBoxLayout *info_layout = new QVBoxLayout();
  QHBoxLayout *left_wheel_vel_layout = new QHBoxLayout();
  QHBoxLayout *right_wheel_vel_layout = new QHBoxLayout();
  QHBoxLayout *row_col_layout = new QHBoxLayout();
  QHBoxLayout *dir_layout = new QHBoxLayout();
  QHBoxLayout *x_layout = new QHBoxLayout();
  QHBoxLayout *y_layout = new QHBoxLayout();
  QHBoxLayout *yaw_layout = new QHBoxLayout();
  QHBoxLayout *buttons_layout = new QHBoxLayout();

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
  main_layout->addLayout(info_layout);
  main_layout->addWidget(maze_edit);

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
