#pragma once

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>

#endif

#include <QtGui/QPainter>
#include <sim/state.pb.h>

namespace gazebo {
  class GAZEBO_VISIBLE StateViz : public GUIPlugin {
  Q_OBJECT

  public:
    StateViz();
    virtual ~StateViz();

  signals: void SetLeftVelocity(QString str);
  signals: void SetRightVelocity(QString str);

  private:
    constexpr static unsigned int WIDTH = 250; // pixels
    constexpr static unsigned int HEIGHT = 40; // pixels

    float left_accumulator, right_accumulator;

    transport::NodePtr node;
    transport::SubscriberPtr state_sub;

    QLabel *left_wheel_velocity_label;
    QLabel *right_wheel_velocity_label;

    QLineEdit *left_wheel_velocity_edit;
    QLineEdit *right_wheel_velocity_edit;

    typedef const boost::shared_ptr<gzmaze::msgs::RobotState const> ConstRobotStatePtr;
    void StateCallback(ConstRobotStatePtr &msg);
  };
}
