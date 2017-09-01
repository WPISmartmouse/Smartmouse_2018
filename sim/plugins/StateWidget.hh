#pragma once

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>

#endif

#include <QtGui/QPainter>
#include <sim/msgs/msgs.h>

namespace gazebo {
  class GAZEBO_VISIBLE SensorState : public QWidget {
  Q_OBJECT

  public:
    SensorState();
    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;

    double frontLeftWall, frontRightWall, backLeftWall, backRightWall, frontWall;

  protected:
    void paintEvent(QPaintEvent *event);

  };

  class GAZEBO_VISIBLE StateViz : public GUIPlugin {
  Q_OBJECT

  public:
    StateViz();

    virtual ~StateViz();

    static constexpr int HEIGHT = 170;
    static constexpr int MAZE_S = HEIGHT - 2;
    static constexpr int WIDTH = 460 + MAZE_S;

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

    void SetMazeEdit(QString str);

    void HighlightX(QString str);
    void HighlightY(QString str);
    void HighlightYaw(QString str);

  protected slots:

    void ClearRobotTrace();

    void StopRobot();

  private:

    void OnStats(ConstWorldStatisticsPtr &msg);

    void FrontLeftAnalogCallback(ConstLaserScanStampedPtr &msg);

    void FrontRightAnalogCallback(ConstLaserScanStampedPtr &msg);

    void BackLeftAnalogCallback(ConstLaserScanStampedPtr &msg);

    void BackRightAnalogCallback(ConstLaserScanStampedPtr &msg);

    void FrontAnalogCallback(ConstLaserScanStampedPtr &msg);

    void StateCallback(ConstRobotStatePtr &msg);

    void MazeLocationCallback(ConstMazeLocationPtr &msg);

    double true_x, true_y, true_yaw;

    std::string topic;

    transport::NodePtr node;
    transport::SubscriberPtr state_sub;
    transport::SubscriberPtr maze_loc_sub;
    transport::SubscriberPtr front_left_analog_sub;
    transport::SubscriberPtr front_right_analog_sub;
    transport::SubscriberPtr back_left_analog_sub;
    transport::SubscriberPtr back_right_analog_sub;
    transport::SubscriberPtr front_analog_sub;
    transport::SubscriberPtr statsSub;
    transport::PublisherPtr stop_pub;
    ignition::transport::Node ign_node;
    ignition::transport::Node::Publisher reset_trace_pub;

    QScrollArea *scroll_area;

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

    QTextEdit *maze_edit;

    SensorState *sensor_state;

    double frontLeftAnalogDist, frontRightAnalogDist, backLeftAnalogDist, backRightAnalogDist, frontAnalogDist;
  };
}
