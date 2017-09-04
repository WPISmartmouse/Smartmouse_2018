#pragma once

#include <time.h>

#include <ignition/transport.hh>
#include <QtWidgets/QMainWindow>

#include <sim/simulator/lib/widgets/StateWidget.h>
#include <sim/simulator/msgs/gui_actions.pb.h>
#include <sim/simulator/msgs/physics_config.pb.h>
#include <sim/simulator/msgs/server_control.pb.h>
#include <sim/simulator/msgs/world_statistics.pb.h>
#include <lib/widgets/MazeWidget.h>

namespace Ui {
class MainWindow;
}

class Client : public QMainWindow {
 Q_OBJECT

 public:
  Client(QMainWindow *parent = 0);

  void closeEvent(QCloseEvent *event) override;

  void OnExit();

  virtual ~Client();

 private slots:

  void Play();

  void Pause();

  void Step();

  void LoadMaze();

  void ShowSourceCode();

  void ShowWiki();

  void RealTimeFactorChanged(double real_time_factor);

  void StepCountChanged(int step_time_ms);

  void TimePerStepMsChanged(int step_time_ms);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "NotImplementedFunctions"
 signals:
  void SetRealTime(QString str);
  void SetTime(QString str);
#pragma clang diagnostic pop

 private:
  void ConfigureGui();

  void OnWorldStats(const smartmouse::msgs::WorldStatistics &msg);
  void OnGuiActions(const smartmouse::msgs::GuiActions &msg);
  void OnPhysics(const smartmouse::msgs::PhysicsConfig &msg);

  ignition::transport::Node node_;
  Ui::MainWindow *ui_;
  ignition::transport::Node::Publisher server_control_pub_;
  ignition::transport::Node::Publisher physics_pub_;

  unsigned int step_count_ = 1u;
  MazeWidget *maze_widget_;
  QString maze_files_dir_;
  QSettings *settings_;
  void writeSettings();
  void RestoreSettings();
};
