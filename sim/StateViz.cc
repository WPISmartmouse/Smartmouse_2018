#include <sstream>
#include <cmath>
#include <boost/algorithm/string/replace.hpp>
#include "StateViz.hh"
#include "RegenerateWidget.hh"
#include "SensorViz.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(StateViz)

StateViz::StateViz() : GUIPlugin() {

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->state_sub = this->node->Subscribe("~/mouse/state", &StateViz::StateCallback, this);

  QVBoxLayout *mainLayout = new QVBoxLayout();

  left_wheel_velocity_label = new QLabel(tr("left wheel velocity"));
  right_wheel_velocity_label = new QLabel(tr("right wheel velocity"));

  left_wheel_velocity_edit = new QLineEdit(tr("0.000 m/sec"));
  right_wheel_velocity_edit = new QLineEdit(tr("0.000 m/sec"));

  connect(this, SIGNAL(SetLeftVelocity(QString)), this->left_wheel_velocity_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetRightVelocity(QString)), this->right_wheel_velocity_edit,
          SLOT(setText(QString)), Qt::QueuedConnection);

  mainLayout->addWidget(left_wheel_velocity_label);
  mainLayout->addWidget(left_wheel_velocity_edit);
  mainLayout->addWidget(right_wheel_velocity_label);
  mainLayout->addWidget(right_wheel_velocity_edit);

  mainLayout->setContentsMargins(2, 2, 2, 2);
  this->setLayout(mainLayout);

  this->move(RegenerateWidget::WIDTH + SensorViz::WIDTH, 0);
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

StateViz::~StateViz() {
}

void StateViz::StateCallback(ConstRobotStatePtr &msg) {
  char left_wheel_velocity_str[12];
  snprintf(left_wheel_velocity_str, 12, "%0.4f m/sec", msg->left_wheel_velocity());

  char right_wheel_velocity_str[12];
  snprintf(right_wheel_velocity_str, 12, "%0.4f m/sec", msg->right_wheel_velocity());

  this->SetLeftVelocity(left_wheel_velocity_str);
  this->SetRightVelocity(right_wheel_velocity_str);
}
