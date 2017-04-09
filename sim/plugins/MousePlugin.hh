#pragma once

using namespace gazebo;

class MousePlugin : public ModelPlugin {
public:

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

  void Update(const common::UpdateInfo &info);

  void FrontLeftAnalogCallback(ConstLaserScanStampedPtr &msg);

  void FrontRightAnalogCallback(ConstLaserScanStampedPtr &msg);

  void BackLeftAnalogCallback(ConstLaserScanStampedPtr &msg);

  void BackRightAnalogCallback(ConstLaserScanStampedPtr &msg);

  void FrontAnalogCallback(ConstLaserScanStampedPtr &msg);

private:

  void PublishInfo();

  physics::ModelPtr model;
  physics::LinkPtr body;
  physics::JointPtr right_wheel;
  physics::JointPtr left_wheel;
  event::ConnectionPtr updateConn;
  transport::NodePtr node;
  transport::PublisherPtr state_pub;
  transport::SubscriberPtr front_left_analog_sub;
  transport::SubscriberPtr front_right_analog_sub;
  transport::SubscriberPtr back_left_analog_sub;
  transport::SubscriberPtr back_right_analog_sub;
  transport::SubscriberPtr front_analog_sub;
  double front_left_analog;
  double front_right_analog;
  double back_left_analog;
  double back_right_analog;
  double front_analog;
};
