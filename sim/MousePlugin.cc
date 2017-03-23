#include <sim/state.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo_core.hh>
#include "MousePlugin.hh"
#include "SimMouse.h"

GZ_REGISTER_MODEL_PLUGIN(MousePlugin)

void MousePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  this->model = model;

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
    this->left_analog = 0.15;
  }
  else {
    this->left_analog = raw_range;
  }
}

void MousePlugin::RightAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);
  if (std::isinf(raw_range)) {
    this->right_analog = 0.15;
  }
  else {
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
  this->right_binary= !std::isinf(range) and range < SimMouse::RIGHT_BINARY_THRESHOLD;
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

  float left_vel = this->left_wheel->GetVelocity(0);
  float right_vel = this->right_wheel->GetVelocity(0);

  gzmaze::msgs::RobotState state;
  state.set_allocated_pose(pose);
  state.set_left_wheel_velocity(left_vel);
  state.set_right_wheel_velocity(right_vel);
  state.set_left_analog(this->left_analog);
  state.set_right_analog(this->right_analog);
  state.set_left_binary(this->left_binary);
  state.set_right_binary(this->right_binary);
  state.set_front_binary(this->front_binary);
  state_pub->Publish(state);

}
