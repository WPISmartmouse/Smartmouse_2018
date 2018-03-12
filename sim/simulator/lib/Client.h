#pragma once

#include <time.h>

#include <ignition/transport.hh>
#include <QtWidgets/QMainWindow>

#include <sim/simulator/lib/widgets/StateWidget.h>
#include <sim/simulator/msgs/gui_actions.pb.h>
#include <sim/simulator/msgs/physics_config.pb.h>
#include <sim/simulator/msgs/server_control.pb.h>
#include <sim/simulator/msgs/world_statistics.pb.h>
#include <sim/simulator/lib/common/sole.hpp>
#include <lib/widgets/MazeWidget.h>

namespace Ui {
class MainWindow;
}

class Client : public QMainWindow {
 Q_OBJECT

 public:
  static const int kRestartCode = 1337;

  Client(QMainWindow *parent = 0);

  void closeEvent(QCloseEvent *event) override;

 public slots:
  void Exit();

 private slots:

  void Restart();
  void TogglePlayPause();
  void SetStatic();
  void ResetMouse();
  void ResetTime();
  void Step();
  void LoadNewMaze();
  void LoadNewMouse();
  void ShowSourceCode();
  void ShowKeyboardShortcuts();
  void ShowWiki();
  void RealTimeFactorChanged(double real_time_factor);
  void StepCountChanged(int step_time_ms);
  void TimePerStepMsChanged(int step_time_ms);
  void SendRobotCmd();
  void SendTeleportCmd();
  void PublishPIDConstants();
  void PublishPIDSetpoints();

#pragma clang diagnostic push
#pragma ide diagnostic ignored "NotImplementedFunctions"
 signals:
  void SetRealTime(QString str);
  void SetTime(QString str);
#pragma clang diagnostic pop

 private:
  void ConfigureGui();
  void LoadDefaultMaze();
  void LoadRandomMaze();
  void LoadDefaultMouse();
  void RestoreSettings();
  void SaveSettings();

  void OnWorldStats(const smartmouse::msgs::WorldStatistics &msg);
  void OnGuiActions(const smartmouse::msgs::GuiActions &msg);
  void OnPhysics(const smartmouse::msgs::PhysicsConfig &msg);
  void OnServerControl(const smartmouse::msgs::ServerControl &msg);

  unsigned int step_count_ = 1u;
  sole::uuid uuid;
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher maze_pub_;
  ignition::transport::Node::Publisher physics_pub_;
  ignition::transport::Node::Publisher server_control_pub_;
  ignition::transport::Node::Publisher robot_command_pub_;
  ignition::transport::Node::Publisher robot_description_pub_;
  ignition::transport::Node::Publisher pid_setpoints_pub_;
  ignition::transport::Node::Publisher pid_constants_pub_;
  MazeWidget *maze_widget_;
  StateWidget *state_widget_;
  QSettings *settings_;
  QString maze_files_dir_;
  QString mouse_files_dir_;
  QString default_maze_file_name_;
  QString default_mouse_file_name_;
  Ui::MainWindow *ui_;
  QShortcut *shortcut;
};
