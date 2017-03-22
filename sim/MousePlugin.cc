#include <sim/state.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo_core.hh>
#include "MousePlugin.hh"

GZ_REGISTER_MODEL_PLUGIN(MousePlugin)

void MousePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  this->model = model;

  body = model->GetLink(sdf->Get<std::string>("body"));
  right_wheel = this->model->GetJoint("right_wheel_joint");
  left_wheel = this->model->GetJoint("left_wheel_joint");

  node = transport::NodePtr(new transport::Node());
  node->Init();
  state_pub = node->Advertise<gzmaze::msgs::RobotState>("~/mouse/state");

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MousePlugin::Update, this, _1));
}

void MousePlugin::Update(const common::UpdateInfo &info) {
  PublishInfo();
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
  state.set_allocated_position(pose);
  state.set_left_wheel(left_vel);
  state.set_right_wheel(right_vel);
  state_pub->Publish(state);

}
