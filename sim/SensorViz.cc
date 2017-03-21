#include <sstream>
#include <boost/algorithm/string/replace.hpp>
#include "SensorViz.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(SensorViz)

SensorViz::SensorViz() : GUIPlugin() {

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->left_analog_sub = this->node->Subscribe("~/mouse/base/left_analog/scan", &SensorViz::LeftAnalogCallback, this);
  this->left_binary_sub = this->node->Subscribe("~/mouse/base/left_binary/scan", &SensorViz::LeftBinaryCallback, this);
  this->right_binary_sub = this->node->Subscribe("~/mouse/base/right_binary/scan", &SensorViz::RightBinaryCallback, this);
  this->right_analog_sub = this->node->Subscribe("~/mouse/base/right_analog/scan", &SensorViz::RightAnalogCallback, this);
}

SensorViz::~SensorViz() {
}

void SensorViz::LeftAnalogCallback(ConstLaserScanStampedPtr &msg) {
  gzmsg << "left analog" << std::endl;
}

void SensorViz::RightAnalogCallback(ConstLaserScanStampedPtr &msg) {
  gzmsg << "right analog" << std::endl;
}

void SensorViz::LeftBinaryCallback(ConstLaserScanStampedPtr &msg) {
  gzmsg << "left binary" << std::endl;
}

void SensorViz::RightBinaryCallback(ConstLaserScanStampedPtr &msg) {
  gzmsg << "right binary" << std::endl;
}
