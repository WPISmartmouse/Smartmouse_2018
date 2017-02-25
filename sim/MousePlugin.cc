#include "MousePlugin.hh"
#include "msgs/msgs.h"

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

void MousePlugin::PublishInfo(){
  math::Pose realtivePose = body->GetWorldPose();

  msgs::Vector3d *pos = new msgs::Vector3d();
  pos->set_x(realtivePose.pos[0]);
  pos->set_y(realtivePose.pos[1]);
  pos->set_z(realtivePose.pos[2]);

  msgs::Quaternion *rot = new msgs::Quaternion();
  rot->set_x(realtivePose.rot.x);
  rot->set_y(realtivePose.rot.y);
  rot->set_z(realtivePose.rot.z);
  rot->set_w(realtivePose.rot.w);

  msgs::Pose *pose = new msgs::Pose();
  pose->set_allocated_position(pos);
  pose->set_allocated_orientation(rot);

  double left_vel = this->left_wheel->GetVelocity(0);
  double right_vel = this->right_wheel->GetVelocity(0);

  gzmaze::msgs::RobotState state;
  state.set_allocated_position(pose);
  state.set_left_wheel(left_vel);
  state.set_right_wheel(right_vel);
  state_pub->Publish(state);

}
