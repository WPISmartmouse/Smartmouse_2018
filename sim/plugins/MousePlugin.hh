#pragma once

using namespace gazebo;

class MousePlugin : public ModelPlugin {
public:

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

  void Update(const common::UpdateInfo &info);

  void LeftAnalogCallback(ConstLaserScanStampedPtr &msg);

  void RightAnalogCallback(ConstLaserScanStampedPtr &msg);

  void LeftBinaryCallback(ConstLaserScanStampedPtr &msg);

  void RightBinaryCallback(ConstLaserScanStampedPtr &msg);

  void FrontBinaryCallback(ConstLaserScanStampedPtr &msg);

  transport::SubscriberPtr left_analog_sub;
  transport::SubscriberPtr right_analog_sub;
  transport::SubscriberPtr left_binary_sub;
  transport::SubscriberPtr right_binary_sub;
  transport::SubscriberPtr front_binary_sub;

private:

  void PublishInfo();

  physics::ModelPtr model;
  physics::LinkPtr body;
  physics::JointPtr right_wheel;
  physics::JointPtr left_wheel;
  event::ConnectionPtr updateConn;
  transport::NodePtr node;
  transport::PublisherPtr state_pub;
  double left_analog;
  double right_analog;
  bool left_binary;
  bool right_binary;
  bool front_binary;
};
