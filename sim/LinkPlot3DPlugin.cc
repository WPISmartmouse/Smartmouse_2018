#include <functional>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "LinkPlot3DPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LinkPlot3DPlugin)

/// \brief Information about each plot
struct Plot3D {
  /// \brief Message
  ignition::msgs::Marker msg;

  /// \brief Link to track
  physics::LinkPtr link;

  /// \brief Pose of the marker
  ignition::math::Pose3d pose;

  /// \brief Store the previous point for distance computation
  ignition::math::Vector3d prevPoint;
};

/// \brief Private data class
class gazebo::LinkPlot3DPluginPrivate {
  /// \brief Connection to World Update events.
public:
  event::ConnectionPtr updateConnection;

  /// \brief Set of plots
public:
  std::vector<Plot3D> plots;

  /// \brief Communication node
public:
  ignition::transport::Node node;

  /// \brief Pointer to the world
public:
  physics::WorldPtr world;

  /// \brief Update period
public:
  int period;

  /// \brief number of points to keep
public:
  int buffer_length;

  /// \brief Update minimum_distance
public:
  float minimum_distance;

  /// \brief PRevious update time.
public:
  common::Time prevTime;

public:
  bool sub;
};

/////////////////////////////////////////////////
LinkPlot3DPlugin::LinkPlot3DPlugin()
        : dataPtr(new LinkPlot3DPluginPrivate) {
}

/////////////////////////////////////////////////
LinkPlot3DPlugin::~LinkPlot3DPlugin() {
}

/////////////////////////////////////////////////
void LinkPlot3DPlugin::Load(physics::ModelPtr _model,
                            sdf::ElementPtr _sdf) {
  GZ_ASSERT(_model, "LinkPlot3DPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LinkPlot3DPlugin _sdf pointer is NULL");

  this->dataPtr->world = _model->GetWorld();

  if (!_sdf->HasElement("plot")) {
    gzwarn << "No plot elements" << std::endl;
    return;
  }

  // minimum distance
  if (_sdf->HasElement("minimum_distance")) {
    this->dataPtr->minimum_distance = _sdf->Get<float>("minimum_distance");
  } else {
    this->dataPtr->minimum_distance = 0.05;
  }

  // buffer length
  if (_sdf->HasElement("buffer_length")) {
    this->dataPtr->buffer_length = _sdf->Get<int>("buffer_length");
  } else {
    this->dataPtr->buffer_length = 1000;
  }
  // Update period
  if (_sdf->HasElement("frequency")) {
    this->dataPtr->period = 1.0 / _sdf->Get<int>("frequency");
  } else {
    this->dataPtr->period = 1.0 / 30.0;
  }

  // Construct the plots
  auto plotElem = _sdf->GetElement("plot");
  int id = 0;
  while (plotElem) {
    auto linkName = plotElem->Get<std::string>("link");

    auto link = _model->GetLink(linkName);

    if (link) {
      Plot3D plot;

      // Link pointer
      plot.link = link;

      // Relative pose to link
      ignition::math::Pose3d p;
      if (plotElem->HasElement("pose"))
        plot.pose = plotElem->Get<ignition::math::Pose3d>("pose");
      else
        plot.pose = ignition::math::Pose3d::Zero;

      // Message
      ignition::msgs::Marker markerMsg;
      markerMsg.set_ns("plot_" + link->GetName());
      markerMsg.set_id(id++);
      markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
      markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
      markerMsg.set_layer(1);

      // Material
      std::string mat;
      if (plotElem->HasElement("material"))
        mat = plotElem->Get<std::string>("material");
      else
        mat = "Gazebo/Black";
      ignition::msgs::Material *matMsg = markerMsg.mutable_material();
      matMsg->mutable_script()->set_name(mat);

      plot.msg = markerMsg;

      this->dataPtr->plots.push_back(plot);
    } else {
      gzerr << "Couldn't find link [" << linkName << "] in model [" <<
            _model->GetName() << "]" << std::endl;
    }

    plotElem = plotElem->GetNextElement("plot");
  }

  if (!this->dataPtr->plots.empty()) {
    this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&LinkPlot3DPlugin::OnUpdate, this));
  }

  // subscribe to requests to clear plot
  std::string topic = "/delete_plot";
  this->dataPtr->sub = this->dataPtr->node.Subscribe(topic, &LinkPlot3DPlugin::delete_plot_callback, this);
  if (! this->dataPtr->sub) {
    gzerr << "Failed to subscribe to [" << topic << "]" << std::endl;
  }
}

/////////////////////////////////////////////////
void LinkPlot3DPlugin::OnUpdate() {
  auto currentTime = this->dataPtr->world->SimTime();

  // check for world reset
  if (currentTime < this->dataPtr->prevTime) {
    this->dataPtr->prevTime = currentTime;
    for (auto &plot : this->dataPtr->plots)
      plot.msg.mutable_point()->Clear();
    return;
  }

  // Throttle update
  if ((currentTime - this->dataPtr->prevTime).Double() < this->dataPtr->period)
    return;

  this->dataPtr->prevTime = currentTime;

  // Process each plot
  for (auto &plot : this->dataPtr->plots) {
    auto point = (plot.pose + plot.link->WorldPose()).Pos();

    // Only add points if the distance is past a threshold.
    if (point.Distance(plot.prevPoint) > this->dataPtr->minimum_distance) {
      plot.prevPoint = point;
      ignition::msgs::Set(plot.msg.add_point(), point);

      // Reduce message array
      if (plot.msg.point_size() > this->dataPtr->buffer_length)
        plot.msg.mutable_point()->DeleteSubrange(0, 5);

      this->dataPtr->node.Request("/marker", plot.msg);
    }
  }
}

void LinkPlot3DPlugin::delete_plot_callback(const ignition::msgs::Empty &_msg)
{
  std::cout << "clear" << std::endl;
  for (auto &plot : this->dataPtr->plots) {
    plot.msg.mutable_point()->Clear();
  }
}

