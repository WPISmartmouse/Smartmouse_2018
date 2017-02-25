#pragma once

using namespace gazebo;

class MousePlugin: public ModelPlugin {
  public:

    /// \brief Load the dc motor and configures it according to the sdf.
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

    /// \brief Update the torque on the joint from the dc motor each timestep.
    void Update(const common::UpdateInfo &info);

  private:

    void PublishInfo();

    physics::ModelPtr model;
    physics::LinkPtr body;
    physics::JointPtr right_wheel;
    physics::JointPtr left_wheel;
    event::ConnectionPtr updateConn;
    transport::NodePtr node;
    transport::PublisherPtr state_pub;
};
