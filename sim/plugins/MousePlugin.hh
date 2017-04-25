#pragma once

using namespace gazebo;

class MousePlugin : public ModelPlugin {
public:

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

  void Update(const common::UpdateInfo &info);

  void GeraldLeftCallback(ConstLaserScanStampedPtr &msg);

  void GeraldRightCallback(ConstLaserScanStampedPtr &msg);

  void FrontLeftCallback(ConstLaserScanStampedPtr &msg);

  void FrontRightCallback(ConstLaserScanStampedPtr &msg);

  void BackLeftCallback(ConstLaserScanStampedPtr &msg);

  void BackRightCallback(ConstLaserScanStampedPtr &msg);

  void FrontCallback(ConstLaserScanStampedPtr &msg);

private:

  void PublishInfo();

  physics::ModelPtr model;
  physics::LinkPtr body;
  physics::JointPtr right_wheel;
  physics::JointPtr left_wheel;
  event::ConnectionPtr updateConn;
  transport::NodePtr node;
  transport::PublisherPtr state_pub;
  transport::SubscriberPtr front_left_sub;
  transport::SubscriberPtr front_right_sub;
  transport::SubscriberPtr gerald_left_sub;
  transport::SubscriberPtr gerald_right_sub;
  transport::SubscriberPtr back_left_sub;
  transport::SubscriberPtr back_right_sub;
  transport::SubscriberPtr front_sub;
  double front_left;
  double front_right;
  double gerald_left;
  double gerald_right;
  double back_left;
  double back_right;
  double front;

  static const double DECAY;
};
