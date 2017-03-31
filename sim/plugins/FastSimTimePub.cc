#include <ignition/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

namespace gazebo {
  class FastSimTimePub : public WorldPlugin {

  public:
    physics::WorldPtr world;
    event::ConnectionPtr updateConn;
    ignition::transport::Node ign_node;
    ignition::transport::Node::Publisher time_pub;

    void Update(const common::UpdateInfo &info) {
      gazebo::common::Time time = this->world->GetSimTime();
      unsigned long time_ms = time.sec * 1000ul + time.nsec / 1000000;

      ignition::msgs::UInt64 msg;
      msg.set_data(time_ms);
      this->time_pub.Publish(msg);
    }

    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
      this->world = _parent;

      this->time_pub = this->ign_node.Advertise<ignition::msgs::UInt64>("time_ms");

      updateConn = event::Events::ConnectWorldUpdateBegin(boost::bind(&FastSimTimePub::Update, this, _1));
    }

  };

  GZ_REGISTER_WORLD_PLUGIN(FastSimTimePub)
}