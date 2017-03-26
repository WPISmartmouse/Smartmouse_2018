#include <sstream>
#include <cmath>
#include <boost/algorithm/string/replace.hpp>
#include <AbstractMaze.h>
#include "StateViz.hh"
#include "RegenerateWidget.hh"
#include "SensorViz.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(StateViz)

StateViz::StateViz() : GUIPlugin(), left_accumulator(0), right_accumulator(0), topic("/delete_plot") {

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->state_sub = this->node->Subscribe("~/mouse/state", &StateViz::StateCallback, this);

  this->pub = this->ign_node.Advertise<ignition::msgs::Empty>(this->topic);
  if (! pub) {
    gzerr << "Failed to advertise to [" << this->topic << "]" << std::endl;
  }

  QVBoxLayout *main_layout = new QVBoxLayout();
  QHBoxLayout *left_wheel_vel_layout = new QHBoxLayout();
  QHBoxLayout *right_wheel_vel_layout= new QHBoxLayout();
  QHBoxLayout *row_layout = new QHBoxLayout();
  QHBoxLayout *col_layout = new QHBoxLayout();

  left_wheel_velocity_label = new QLabel(tr("left wheel velocity:"));
  right_wheel_velocity_label = new QLabel(tr("right wheel velocity:"));
  row_label = new QLabel(tr("row:"));
  col_label = new QLabel(tr("column:"));

  left_wheel_velocity_edit = new QLineEdit(tr("0.000 m/sec"));
  right_wheel_velocity_edit = new QLineEdit(tr("0.000 m/sec"));
  row_edit = new QLineEdit(tr("0"));
  col_edit = new QLineEdit(tr("0"));

  QPushButton *clear_plot_button = new QPushButton(tr("Clear Robot Trace"));
  clear_plot_button->setStyleSheet("padding: 0px;");
  connect(clear_plot_button, SIGNAL(clicked()), this, SLOT(ClearRobotTrace()));

  connect(this, SIGNAL(SetLeftVelocity(QString)), this->left_wheel_velocity_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetRightVelocity(QString)), this->right_wheel_velocity_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetRow(QString)), this->row_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetCol(QString)), this->col_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);

  left_wheel_vel_layout->addWidget(left_wheel_velocity_label);
  left_wheel_vel_layout->addWidget(left_wheel_velocity_edit);
  right_wheel_vel_layout->addWidget(right_wheel_velocity_label);
  right_wheel_vel_layout->addWidget(right_wheel_velocity_edit);
  row_layout->addWidget(row_label);
  row_layout->addWidget(row_edit);
  col_layout->addWidget(col_label);
  col_layout->addWidget(col_edit);
  main_layout->addLayout(left_wheel_vel_layout);
  main_layout->addLayout(right_wheel_vel_layout);
  main_layout->addLayout(row_layout);
  main_layout->addLayout(col_layout);
  main_layout->addWidget(clear_plot_button);

  QPalette pal = palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  this->setAutoFillBackground(true);
  this->setPalette(pal);

  main_layout->setContentsMargins(2, 2, 2, 2);
  this->setLayout(main_layout);

  this->move(RegenerateWidget::WIDTH + SensorViz::WIDTH, 0);
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

StateViz::~StateViz() {
}

void StateViz::StateCallback(ConstRobotStatePtr &msg) {
  constexpr double WHEEL_RAD = 0.015;
  constexpr double WHEEL_CIRC = 2 * WHEEL_RAD * M_PI;

  double smooth_left_vel = (0.9 * left_accumulator + 0.1 * msg->left_wheel_velocity()) * WHEEL_CIRC / (2 * M_PI);
  double smooth_right_vel = (0.9 * right_accumulator + 0.1 * msg->right_wheel_velocity()) * WHEEL_CIRC / (2 * M_PI);

  char left_wheel_velocity_str[11];
  snprintf(left_wheel_velocity_str, 11, "%2.2f mm/s", (1000 * smooth_left_vel));

  char right_wheel_velocity_str[11];
  snprintf(right_wheel_velocity_str, 11, "%2.2f mm/s", (1000 * smooth_right_vel));

  gazebo::msgs::Pose pose = msg->pose();

  // compute x and y with respect to the top left square
  double x = pose.position().x() + AbstractMaze::UNIT_DIST * 8;
  double y = -pose.position().y() + AbstractMaze::UNIT_DIST * 8;

  int row = (int) (y / (AbstractMaze::UNIT_DIST));
  int col = (int) (x / (AbstractMaze::UNIT_DIST));
  double row_offset = fmod(y, AbstractMaze::UNIT_DIST) - AbstractMaze::UNIT_DIST/2;
  double col_offset = fmod(x, AbstractMaze::UNIT_DIST) - AbstractMaze::UNIT_DIST/2;

  char row_str[14];
  snprintf(row_str, 14, "%i (%0.3f m)", row, row_offset);
  char col_str[14];
  snprintf(col_str, 14, "%i (%0.3f m)", col, col_offset);
  this->SetRow(row_str);
  this->SetCol(col_str);

  this->last_pose = pose;

  this->SetLeftVelocity(left_wheel_velocity_str);
  this->SetRightVelocity(right_wheel_velocity_str);
}

void StateViz::ClearRobotTrace() {
  ignition::msgs::Empty msg;
  this->pub.Publish(msg);
}
