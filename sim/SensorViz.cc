#include <sstream>
#include <cmath>
#include <boost/algorithm/string/replace.hpp>
#include "SensorViz.hh"
#include "RegenerateWidget.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(SensorViz)

SensorViz::SensorViz() : GUIPlugin(),
                         leftWall(false),
                         rightWall(false),
                         frontWall(false),
                         leftAnalogDist(0),
                         rightAnalogDist(0),
                         leftBinaryState(false),
                         rightBinaryState(false),
                         frontBinaryState(false) {
  this->move(RegenerateWidget::WIDTH + 2, 0);
  this->resize(SensorViz::WIDTH, SensorViz::HEIGHT);
  QPalette pal = palette();

  // set black background
  pal.setColor(QPalette::Background, Qt::lightGray);
  this->setAutoFillBackground(true);
  this->setPalette(pal);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->left_analog_sub = this->node->Subscribe("~/mouse/base/left_analog/scan", &SensorViz::LeftAnalogCallback, this);
  this->right_analog_sub = this->node->Subscribe("~/mouse/base/right_analog/scan", &SensorViz::RightAnalogCallback,
                                                 this);
  this->left_binary_sub = this->node->Subscribe("~/mouse/base/left_binary/scan", &SensorViz::LeftBinaryCallback, this);
  this->right_binary_sub = this->node->Subscribe("~/mouse/base/right_binary/scan", &SensorViz::RightBinaryCallback,
                                                 this);
  this->front_binary_sub = this->node->Subscribe("~/mouse/base/front_binary/scan", &SensorViz::FrontBinaryCallback,
                                                 this);
  this->statsSub = this->node->Subscribe("~/world_stats",
                                         &SensorViz::OnStats, this);

//  QTimer *timer = new QTimer(this);
//  connect(timer, SIGNAL(timeout()), this, SLOT(Update()));
//  timer->start(30); // 100 millisecond period
}

SensorViz::~SensorViz() {
}

void SensorViz::OnStats(ConstWorldStatisticsPtr &msg) {
  // pass
  update();
}

void SensorViz::LeftAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);
  if (std::isinf(raw_range)) {
    this->leftAnalogDist = 0.15;
  }
  else {
    this->leftAnalogDist = raw_range;
  }
}

void SensorViz::RightAnalogCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double raw_range = scan.ranges(0);
  if (std::isinf(raw_range)) {
    this->rightAnalogDist = 0.15;
  }
  else {
    this->rightAnalogDist = raw_range;
  }
}

void SensorViz::LeftBinaryCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double range = scan.ranges(0);
  this->leftBinaryState = !std::isinf(range) and range < LEFT_BINARY_THRESHOLD;
}

void SensorViz::RightBinaryCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double range = scan.ranges(0);
  this->rightBinaryState = !std::isinf(range) and range < RIGHT_BINARY_THRESHOLD;
}

void SensorViz::FrontBinaryCallback(ConstLaserScanStampedPtr &msg) {
  msgs::LaserScan scan = msg->scan();
  assert(scan.ranges_size() == 1);
  double range = scan.ranges(0);
  this->frontBinaryState = !std::isinf(range) and range < FRONT_BINARY_THRESHOLD;
}

void SensorViz::paintEvent(QPaintEvent *event) {
  int mouse_x = 38;
  int mouse_y = SensorViz::HEIGHT / 2;

  QPainter painter(this);

  // draw mouse
  QPainterPath mousePath;
  mousePath.moveTo(mouse_x - 16, mouse_y - 20);
  mousePath.lineTo(mouse_x + 24, mouse_y - 20);
  mousePath.lineTo(mouse_x + 34, mouse_y - 10);
  mousePath.lineTo(mouse_x + 34, mouse_y + 10);
  mousePath.lineTo(mouse_x + 24, mouse_y + 20);
  mousePath.lineTo(mouse_x - 16, mouse_y + 20);
  mousePath.lineTo(mouse_x - 26, mouse_y + 15);
  mousePath.lineTo(mouse_x - 26, mouse_y - 15);
  mousePath.lineTo(mouse_x - 16, mouse_y - 20);
  mousePath.closeSubpath();
  painter.setPen(QPen(Qt::black));
  painter.setBrush(QBrush(Qt::yellow));
  painter.drawPath(mousePath);
  painter.setBrush(QBrush(Qt::black));
  painter.drawRect(mouse_x - 9, mouse_y - 20, 18, 8);
  painter.drawRect(mouse_x - 9, mouse_y + 12, 18, 8);

  // draw the walls
  painter.setPen(QPen(Qt::white));
  painter.setBrush(QBrush(Qt::white));
  if (this->leftWall) {
    painter.drawRect(0, 0, WIDTH, 9);
  }
  if (this->rightWall) {
    painter.drawRect(0, HEIGHT - 9, WIDTH, HEIGHT);
  }
  if (this->frontWall) {
    painter.drawRect(WIDTH - 9, 0, WIDTH, HEIGHT);
  }

  // rays for analog sensors
  int left_analog_x = (int) (cos(ANALOG_ANGLE) * leftAnalogDist * meters_to_pixels);
  int left_analog_y = (int) (sin(ANALOG_ANGLE) * leftAnalogDist * meters_to_pixels);
  painter.setPen(QPen(Qt::blue));
  painter.drawLine(mouse_x + 30, mouse_y - 10, mouse_x + 30 + left_analog_x, mouse_y - 10 - left_analog_y);

  int right_analog_x = (int) (cos(ANALOG_ANGLE) * rightAnalogDist * meters_to_pixels);
  int right_analog_y = (int) (sin(ANALOG_ANGLE) * rightAnalogDist * meters_to_pixels);
  painter.setPen(QPen(Qt::blue));
  painter.drawLine(mouse_x + 30, mouse_y + 10, mouse_x + 30 + right_analog_x, mouse_y + 10 + right_analog_y);

  // rays for binary sensors
  if (this->leftBinaryState) {
    painter.setPen(QPen(Qt::green));
  }
  else {
    painter.setPen(QPen(Qt::darkGray));
  }
  const int bin_l = (int) (0.18 * meters_to_pixels); // fairly arbitrary
  int left_binary_x = (int) (cos(BINARY_ANGLE) * bin_l);
  int left_binary_y = (int) (sin(BINARY_ANGLE) * bin_l);
  painter.drawLine(mouse_x + 32, mouse_y - 8, mouse_x + 32 + left_binary_x, mouse_y - 8 - left_binary_y);

  if (this->rightBinaryState) {
    painter.setPen(QPen(Qt::green));
  }
  else {
    painter.setPen(QPen(Qt::darkGray));
  }
  int right_binary_x = (int) (cos(BINARY_ANGLE) * bin_l);
  int right_binary_y = (int) (sin(BINARY_ANGLE) * bin_l);
  painter.drawLine(mouse_x + 32, mouse_y + 8, mouse_x + 32 + right_binary_x , mouse_y + 8 + right_binary_y);

  if (this->frontBinaryState) {
    painter.setPen(QPen(Qt::green));
  }
  else {
    painter.setPen(QPen(Qt::darkGray));
  }
  painter.drawLine(mouse_x + 32, mouse_y, mouse_x + 32 + bin_l, mouse_y);

  // text for analog sensors
  char leftDistStr[6];
  snprintf(leftDistStr, 6, "%0.2f m" ,this->leftAnalogDist);
  painter.setPen(QPen(Qt::black));
  painter.drawText(30, 30, leftDistStr);

  char rightDistStr[6];
  snprintf(rightDistStr, 6, "%0.2f m" ,this->rightAnalogDist);
  painter.setPen(QPen(Qt::black));
  painter.drawText(30, HEIGHT - 20, rightDistStr);
}
