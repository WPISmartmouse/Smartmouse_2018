#include <sim/state.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo_core.hh>
#include <sim/lib/SimMouse.h>

#include "MousePlugin.hh"

GZ_REGISTER_MODEL_PLUGIN(MousePlugin)

const double MousePlugin::DECAY = 0.1;

void MousePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  this->model = model;

  body = model->GetLink(sdf->Get<std::string>("link"));
  right_wheel = this->model->GetJoint("right_wheel_joint");
  left_wheel = this->model->GetJoint("left_wheel_joint");

  node = transport::NodePtr(new transport::Node());
  node->Init();
  state_pub = node->Advertise<gzmaze::msgs::RobotState>("~/mouse/state");

  this->front_left_sub = this->node->Subscribe("~/mouse/base/front_left/scan", &MousePlugin::FrontLeftCallback, this);
  this->front_right_sub = this->node->Subscribe("~/mouse/base/front_right/scan", &MousePlugin::FrontRightCallback, this);
  this->gerald_left_sub = this->node->Subscribe("~/mouse/base/gerald_left/scan", &MousePlugin::GeraldLeftCallback, this);
  this->gerald_right_sub = this->node->Subscribe("~/mouse/base/gerald_right/scan", &MousePlugin::GeraldRightCallback, this);
  this->back_left_sub = this->node->Subscribe("~/mouse/base/back_left/scan", &MousePlugin::BackLeftCallback, this);
  this->back_right_sub = this->node->Subscribe("~/mouse/base/back_right/scan", &MousePlugin::BackRightCallback, this);
  this->front_sub = this->node->Subscribe("~/mouse/base/front/scan", &MousePlugin::FrontCallback, this);

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = event::Events::ConnectWorldUpdateBegin( boost::bind(&MousePlugin::Update, this, _1));
}

void MousePlugin::Update(const common::UpdateInfo &info) {
  PublishInfo();
}

void MousePlugin::FrontLeftCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double front_left;
  if (std::isinf(raw_range)) {
    front_left = config.ANALOG_MAX_DIST;
  } else {
    front_left = raw_range;
  }

  // to simulate the gradual drop-off of real sensors, use a cheap accumulator
  this->front_left = DECAY * this->front_left + (1 - DECAY) * front_left;
}

void MousePlugin::FrontRightCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double front_right;
  if (std::isinf(raw_range)) {
    front_right = config.ANALOG_MAX_DIST;
  } else {
    front_right = raw_range;
  }

  this->front_right = DECAY * this->front_right + (1 - DECAY) * front_right;
}

void MousePlugin::GeraldLeftCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double gerald_left;
  if (std::isinf(raw_range)) {
    gerald_left = config.ANALOG_MAX_DIST;
  } else {
    gerald_left = raw_range;
  }

  // to simulate the gradual drop-off of real sensors, use a cheap accumulator
  this->gerald_left = DECAY * this->gerald_left + (1 - DECAY) * gerald_left;
}

void MousePlugin::GeraldRightCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double gerald_right;
  if (std::isinf(raw_range)) {
    gerald_right = config.ANALOG_MAX_DIST;
  } else {
    gerald_right = raw_range;
  }

  this->gerald_right = DECAY * this->gerald_right + (1 - DECAY) * gerald_right;
}

void MousePlugin::BackLeftCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double back_left;
  if (std::isinf(raw_range)) {
    back_left = config.ANALOG_MAX_DIST;
  } else {
    back_left = raw_range;
  }

  this->back_left = DECAY * this->back_left + (1 - DECAY) * back_left;
}

void MousePlugin::BackRightCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double back_right;
  if (std::isinf(raw_range)) {
    back_right = config.ANALOG_MAX_DIST;
  } else {
    back_right = raw_range;
  }

  this->back_right = DECAY * this->back_right + (1 - DECAY) * back_right;
}

void MousePlugin::FrontCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double front;
  if (std::isinf(raw_range)) {
    front = config.ANALOG_MAX_DIST;
  } else {
    front = raw_range;
  }

  this->front = DECAY * this->front + (1 - DECAY) * front;
}

void MousePlugin::PublishInfo() {
  ignition::math::Pose3d relativePose = body->WorldCoGPose();

  msgs::Vector3d *pos = new msgs::Vector3d();
  pos->set_x(relativePose.Pos()[0]);
  pos->set_y(relativePose.Pos()[1]);
  pos->set_z(relativePose.Pos()[2]);

  msgs::Quaternion *rot = new msgs::Quaternion();
  rot->set_x(relativePose.Rot().X());
  rot->set_y(relativePose.Rot().Y());
  rot->set_z(relativePose.Rot().Z());
  rot->set_w(relativePose.Rot().W());

  msgs::Pose *pose = new msgs::Pose();
  pose->set_allocated_position(pos);
  pose->set_allocated_orientation(rot);

  float left_vel_rps = 0;
  float right_vel_rps = 0;
  float left_angle = 0;
  float right_angle = 0;
  left_vel_rps = this->left_wheel->GetVelocity(0);
  right_vel_rps = this->right_wheel->GetVelocity(0);
  left_angle = this->left_wheel->GetAngle(0).Radian();
  right_angle = this->right_wheel->GetAngle(0).Radian();

  double left_vel_mps = SimMouse::radToMeters(left_vel_rps);
  double right_vel_mps = SimMouse::radToMeters(right_vel_rps);

  gzmaze::msgs::RobotState state;
  state.set_allocated_true_pose(pose);
  state.set_left_wheel_velocity_mps(left_vel_mps);
  state.set_right_wheel_velocity_mps(right_vel_mps);
  state.set_left_wheel_angle_radians(left_angle);
  state.set_right_wheel_angle_radians(right_angle);
  state.set_front_left(this->front_left);
  state.set_front_right(this->front_right);
  state.set_gerald_left(this->gerald_left);
  state.set_gerald_right(this->gerald_right);
  state.set_back_left(this->back_left);
  state.set_back_right(this->back_right);
  state.set_front(this->front);
  state.set_true_x_meters(pos->x());
  state.set_true_y_meters(-pos->y());
  double yaw = relativePose.Rot().Euler()[2];
  state.set_true_yaw_rad(yaw);
  state_pub->Publish(state);
}
