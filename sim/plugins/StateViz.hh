#pragma once

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>

#endif

#include <QtGui/QPainter>
#include <sim/msgs/msgs.h>

namespace gazebo {
  class GAZEBO_VISIBLE StateViz : public GUIPlugin {
  Q_OBJECT

  public:
    StateViz();

    virtual ~StateViz();

  signals:

    void SetLeftVelocity(QString str);

    void SetRightVelocity(QString str);

    void SetRow(QString str);

    void SetCol(QString str);

    void SetDir(QString str);

    void SetTrueX(QString str);

    void SetTrueY(QString str);

    void SetTrueYaw(QString str);

    void SetEstimatedX(QString str);

    void SetEstimatedY(QString str);

    void SetEstimatedYaw(QString str);

  protected slots:

    void ClearRobotTrace();

    void StopRobot();

  private:
    gazebo::msgs::Pose last_pose;

    std::string topic;

    transport::NodePtr node;
    transport::SubscriberPtr state_sub;
    transport::SubscriberPtr maze_loc_sub;
    transport::PublisherPtr stop_pub;
    ignition::transport::Node ign_node;
    ignition::transport::Node::Publisher reset_trace_pub;

    QLabel *left_wheel_velocity_label;
    QLabel *right_wheel_velocity_label;
    QLabel *col_label;
    QLabel *row_label;
    QLabel *dir_label;
    QLabel *true_x_label;
    QLabel *true_y_label;
    QLabel *true_yaw_label;
    QLabel *estimated_x_label;
    QLabel *estimated_y_label;
    QLabel *estimated_yaw_label;

    QLineEdit *left_wheel_velocity_edit;
    QLineEdit *right_wheel_velocity_edit;
    QLineEdit *col_edit;
    QLineEdit *row_edit;
    QLineEdit *dir_edit;
    QLineEdit *true_x_edit;
    QLineEdit *true_y_edit;
    QLineEdit *true_yaw_edit;
    QLineEdit *estimated_x_edit;
    QLineEdit *estimated_y_edit;
    QLineEdit *estimated_yaw_edit;

    void StateCallback(ConstRobotStatePtr &msg);

    void MazeLocationCallback(ConstMazeLocationPtr &msg);
  };
}
