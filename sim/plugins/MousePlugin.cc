#include <sim/state.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo_core.hh>
#include <sim/SimMouse.h>

#include "MousePlugin.hh"

GZ_REGISTER_MODEL_PLUGIN(MousePlugin)


void MousePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  this->model = model;
  this->left_accumulator = 0;
  this->right_accumulator = 0;

  body = model->GetLink(sdf->Get<std::string>("body"));
  right_wheel = this->model->GetJoint("right_wheel_joint");
  left_wheel = this->model->GetJoint("left_wheel_joint");

  node = transport::NodePtr(new transport::Node());
  node->Init();
  state_pub = node->Advertise<gzmaze::msgs::RobotState>("~/mouse/state");

  this->left_analog_sub = node->Subscribe("~/mouse/base/left_analog/scan", &MousePlugin::LeftAnalogCallback, this);
  this->right_analog_sub = node->Subscribe("~/mouse/base/right_analog/scan", &MousePlugin::RightAnalogCallback,
                                           this);
  this->left_binary_sub = node->Subscribe("~/mouse/base/left_binary/scan", &MousePlugin::LeftBinaryCallback, this);
  this->right_binary_sub = node->Subscribe("~/mouse/base/right_binary/scan", &MousePlugin::RightBinaryCallback,
                                           this);
  this->front_binary_sub = node->Subscribe("~/mouse/base/front_binary/scan", &MousePlugin::FrontBinaryCallback,
                                           this);

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MousePlugin::Update, this, _1));
}

void MousePlugin::Update(const common::UpdateInfo &info) {
  PublishInfo();
}

void MousePlugin::LeftAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);

  if (std::isinf(raw_range)) {
    this->left_analog = SimMouse::ANALOG_MAX_DIST;
  } else {
    this->left_analog = raw_range;
  }
}

void MousePlugin::RightAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);
  if (std::isinf(raw_range)) {
    this->right_analog = SimMouse::ANALOG_MAX_DIST;
  } else {
    this->right_analog = raw_range;
  }
}

void MousePlugin::LeftBinaryCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double range = scan.ranges(0);
  this->left_binary = !std::isinf(range) and range < SimMouse::LEFT_BINARY_THRESHOLD;
}

void MousePlugin::RightBinaryCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double range = scan.ranges(0);
  this->right_binary = !std::isinf(range) and range < SimMouse::RIGHT_BINARY_THRESHOLD;
}

void MousePlugin::FrontBinaryCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double range = scan.ranges(0);
  this->front_binary = !std::isinf(range) and range < SimMouse::FRONT_BINARY_THRESHOLD;
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
  if (this->left_wheel && this->right_wheel) {
    left_vel_rps = this->left_wheel->GetVelocity(0);
    right_vel_rps = this->right_wheel->GetVelocity(0);
    left_angle = this->left_wheel->GetAngle(0).Radian();
    right_angle = this->right_wheel->GetAngle(0).Radian();
  }

  double left_vel_mps = SimMouse::radPerSecToMetersPerSec(left_vel_rps);
  double right_vel_mps = SimMouse::radPerSecToMetersPerSec(right_vel_rps);

  double smooth_left_vel_mps = (RegulatedMotor::VEL_SMOOTHING * left_accumulator) + ((1 - RegulatedMotor::VEL_SMOOTHING) * left_vel_mps);
  double smooth_right_vel_mps = (RegulatedMotor::VEL_SMOOTHING * right_accumulator) + ((1 - RegulatedMotor::VEL_SMOOTHING) * right_vel_mps);

  left_accumulator = smooth_left_vel_mps;
  right_accumulator = smooth_right_vel_mps;

  gzmaze::msgs::RobotState state;
  state.set_allocated_true_pose(pose);
  state.set_left_wheel_velocity_mps(smooth_left_vel_mps);
  state.set_right_wheel_velocity_mps(smooth_right_vel_mps);
  state.set_left_wheel_angle_radians(left_angle);
  state.set_right_wheel_angle_radians(right_angle);
  state.set_left_analog(this->left_analog);
  state.set_right_analog(this->right_analog);
  state.set_left_binary(this->left_binary);
  state.set_right_binary(this->right_binary);
  state.set_front_binary(this->front_binary);
  state.set_true_x_meters(pos->x());
  state.set_true_y_meters(-pos->y());
  double yaw = relativePose.Rot().Euler()[2];
  state.set_true_yaw_rad(yaw);
  state_pub->Publish(state);
}
