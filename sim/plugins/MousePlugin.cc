#include <sim/state.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo_core.hh>
#include <sim/SimMouse.h>

#include "MousePlugin.hh"

GZ_REGISTER_MODEL_PLUGIN(MousePlugin)


void MousePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  this->model = model;

  body = model->GetLink(sdf->Get<std::string>("link"));
  right_wheel = this->model->GetJoint("right_wheel_joint");
  left_wheel = this->model->GetJoint("left_wheel_joint");

  node = transport::NodePtr(new transport::Node());
  node->Init();
  state_pub = node->Advertise<gzmaze::msgs::RobotState>("~/mouse/state");

  this->front_left_analog_sub = this->node->Subscribe("~/mouse/base/front_left/scan", &MousePlugin::FrontLeftAnalogCallback, this);
  this->front_right_analog_sub = this->node->Subscribe("~/mouse/base/front_right/scan", &MousePlugin::FrontRightAnalogCallback, this);
  this->back_left_analog_sub = this->node->Subscribe("~/mouse/base/back_left/scan", &MousePlugin::BackLeftAnalogCallback, this);
  this->back_right_analog_sub = this->node->Subscribe("~/mouse/base/back_right/scan", &MousePlugin::BackRightAnalogCallback, this);
  this->front_analog_sub = this->node->Subscribe("~/mouse/base/front/scan", &MousePlugin::FrontAnalogCallback, this);

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = event::Events::ConnectWorldUpdateBegin( boost::bind(&MousePlugin::Update, this, _1));
}

void MousePlugin::Update(const common::UpdateInfo &info) {
  PublishInfo();
}

void MousePlugin::FrontLeftAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double front_left_analog;
  if (std::isinf(raw_range)) {
    front_left_analog = config.ANALOG_MAX_DIST;
  } else {
    front_left_analog = raw_range;
  }

  // to simulate the gradual drop-off of real sensors, use a cheap accumulator
  this->front_left_analog = 0.5 * this->front_left_analog + 0.5 * front_left_analog;
}

void MousePlugin::FrontRightAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double front_right_analog;
  if (std::isinf(raw_range)) {
    front_right_analog = config.ANALOG_MAX_DIST;
  } else {
    front_right_analog = raw_range;
  }

  this->front_right_analog = 0.5 * this->front_right_analog + 0.5 * front_right_analog;
}

void MousePlugin::BackLeftAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double back_left_analog;
  if (std::isinf(raw_range)) {
    back_left_analog = config.ANALOG_MAX_DIST;
  } else {
    back_left_analog = raw_range;
  }

  this->back_left_analog = 0.5 * this->back_left_analog + 0.5 * back_left_analog;
}

void MousePlugin::BackRightAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double back_right_analog;
  if (std::isinf(raw_range)) {
    back_right_analog = config.ANALOG_MAX_DIST;
  } else {
    back_right_analog = raw_range;
  }

  this->back_right_analog = 0.5 * this->back_right_analog + 0.5 * back_right_analog;
}

void MousePlugin::FrontAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  double front_analog;
  if (std::isinf(raw_range)) {
    front_analog = config.ANALOG_MAX_DIST;
  } else {
    front_analog = raw_range;
  }

  this->front_analog = 0.5 * this->front_analog + 0.5 * front_analog;
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
  state.set_front_left_analog(this->front_left_analog);
  state.set_front_right_analog(this->front_right_analog);
  state.set_back_left_analog(this->back_left_analog);
  state.set_back_right_analog(this->back_right_analog);
  state.set_front_analog(this->front_analog);
  state.set_true_x_meters(pos->x());
  state.set_true_y_meters(-pos->y());
  double yaw = relativePose.Rot().Euler()[2];
  state.set_true_yaw_rad(yaw);
  state_pub->Publish(state);
}
