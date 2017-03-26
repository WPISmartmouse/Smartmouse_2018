#pragma once

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>

#endif

#include <QtGui/QPainter>

namespace gazebo {
  class GAZEBO_VISIBLE SensorViz : public GUIPlugin {
  Q_OBJECT

  public:
    constexpr static unsigned int WIDTH = 200; // pixels
    constexpr static unsigned int HEIGHT = 120; // pixels

    SensorViz();
    virtual ~SensorViz();

  protected:
    void paintEvent(QPaintEvent *event);

  private:
    constexpr static double meters_to_pixels = 625;
    transport::NodePtr node;
    transport::SubscriberPtr left_analog_sub;
    transport::SubscriberPtr right_analog_sub;
    transport::SubscriberPtr left_binary_sub;
    transport::SubscriberPtr right_binary_sub;
    transport::SubscriberPtr front_binary_sub;
    transport::SubscriberPtr statsSub;

    bool leftWall, rightWall, frontWall;
    double leftAnalogDist, rightAnalogDist;
    bool leftBinaryState, rightBinaryState, frontBinaryState;

    void LeftAnalogCallback(ConstLaserScanStampedPtr &msg);
    void RightAnalogCallback(ConstLaserScanStampedPtr &msg);
    void LeftBinaryCallback(ConstLaserScanStampedPtr &msg);
    void RightBinaryCallback(ConstLaserScanStampedPtr &msg);
    void FrontBinaryCallback(ConstLaserScanStampedPtr &msg);
    void OnStats(ConstWorldStatisticsPtr &msg);
  };
}
