#pragma once

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>
#include <QtGui/QPainter>

#endif


namespace gazebo {
  class GAZEBO_VISIBLE SensorViz : public GUIPlugin {
  Q_OBJECT

  public:
    SensorViz();
    virtual ~SensorViz();

  protected:
    void paintEvent(QPaintEvent *event);

  private:
    unsigned int counter;
    transport::NodePtr node;
    transport::SubscriberPtr left_analog_sub;
    transport::SubscriberPtr right_analog_sub;
    transport::SubscriberPtr left_binary_sub;
    transport::SubscriberPtr right_binary_sub;

    void LeftAnalogCallback(ConstLaserScanStampedPtr &msg);
    void RightAnalogCallback(ConstLaserScanStampedPtr &msg);
    void LeftBinaryCallback(ConstLaserScanStampedPtr &msg);
    void RightBinaryCallback(ConstLaserScanStampedPtr &msg);
  };
}
