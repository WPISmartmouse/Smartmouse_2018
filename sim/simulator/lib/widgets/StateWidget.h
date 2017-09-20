#pragma once

#include <QtGui/QPainter>
#include <QLineEdit>
#include <QTextEdit>
#include <QScrollArea>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <ignition/transport/Node.hh>
#include <QtWidgets/QLabel>

#include <sim/simulator/msgs/maze_location.pb.h>
#include <sim/simulator/msgs/robot_sim_state.pb.h>
#include <sim/simulator/lib/widgets/AbstractTab.h>
#include <sim/simulator/lib/widgets/SensorStateWidget.h>

namespace Ui {
class StateWidget;
}

class StateWidget : public AbstractTab {
 Q_OBJECT

 public:
  StateWidget();

  const QString getTabName() override;

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

  void HighlightX(QString str);
  void HighlightY(QString str);
  void HighlightYaw(QString str);
#pragma clang diagnostic pop

 private:

  void OnStats(const ignition::msgs::WorldStatistics &msg);
  void StateCallback(const smartmouse::msgs::RobotSimState &msg);
  void MazeLocationCallback(const smartmouse::msgs::MazeLocation &msg);

  double true_x, true_y, true_yaw;

  ignition::transport::Node node;

  SensorState *sensor_state;

  Ui::StateWidget *ui_;
};
