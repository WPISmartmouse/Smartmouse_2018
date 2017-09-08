#pragma once

#include <QtGui/QPainter>
#include <QLineEdit>
#include <QTextEdit>
#include <QScrollArea>
#include <ignition/transport/Node.hh>
#include <QtWidgets/QLabel>
#include <msgs/maze_location.pb.h>
#include <msgs/robot_sim_state.pb.h>

class SensorState : public QWidget {
 Q_OBJECT

 public:
  SensorState();
  QSize minimumSizeHint() const override;
  QSize sizeHint() const override;

  double frontLeftWall, frontRightWall, backLeftWall, backRightWall, frontWall;

 protected:
  void paintEvent(QPaintEvent *event);

};

class StateWidget : public QWidget {
 Q_OBJECT

 public:
  StateWidget();

  static constexpr int HEIGHT = 170;
  static constexpr int MAZE_S = HEIGHT - 2;
  static constexpr int WIDTH = 460 + MAZE_S;

 signals:

#pragma clang diagnostic push
#pragma ide diagnostic ignored "NotImplementedFunctions"
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
#pragma clang diagnostic pop

 protected slots:

  void ClearRobotTrace();

  void StopRobot();

 private:

  void OnStats(const ignition::msgs::WorldStatistics &msg);

  void FrontLeftAnalogCallback(const ignition::msgs::LaserScanStamped &msg);

  void FrontRightAnalogCallback(const ignition::msgs::LaserScanStamped &msg);

  void BackLeftAnalogCallback(const ignition::msgs::LaserScanStamped &msg);

  void BackRightAnalogCallback(const ignition::msgs::LaserScanStamped &msg);

  void FrontAnalogCallback(const ignition::msgs::LaserScanStamped &msg);

  void StateCallback(const smartmouse::msgs::RobotSimState &msg);

  void MazeLocationCallback(const smartmouse::msgs::MazeLocation &msg);

  double true_x, true_y, true_yaw;

  std::string topic;

  ignition::transport::Node node;
  ignition::transport::Node::Publisher stop_pub;
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
};
