#pragma once
#include <time.h>

#include <ignition/transport.hh>
#include <QWidget>
#include <QtWidgets/QMainWindow>

#include <sim/simulator/lib/widgets/StateWidget.h>
#include <sim/simulator/lib/widgets/WorldWidget.h>
#include <sim/simulator/msgs/gui_actions.pb.h>
#include <sim/simulator/msgs/physics_config.pb.h>
#include <sim/simulator/msgs/server_control.pb.h>
#include <sim/simulator/msgs/world_statistics.pb.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
 Q_OBJECT

 public:
  MainWindow(QMainWindow *parent = 0);

  void OnExit();

  virtual ~MainWindow();

 private slots:

  void Play();

  void Pause();

  void Step();

  void ShowSourceCode();

  void ShowWiki();

  void RealTimeFactorChanged(double real_time_factor);

  void StepCountChanged(int step_time_ms);

  void StepTimeMsChanged(int step_time_ms);

 private:
  void OnWorldControl(const smartmouse::msgs::ServerControl &msg);
  void OnWorldStats(const smartmouse::msgs::WorldStatistics &msg);
  void OnGuiActions(const smartmouse::msgs::GuiActions &msg);
  void OnPhysics(const smartmouse::msgs::PhysicsConfig &msg);

  ignition::transport::Node node_;
  Ui::MainWindow *ui_;
  ignition::transport::Node::Publisher server_control_pub_;
  ignition::transport::Node::Publisher physics_pub_;

  WorldWidget *world_widget_;
  unsigned int step_count_ = 1u;
};
