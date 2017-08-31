#pragma once
#include <time.h>

#include <ignition/transport.hh>
#include <QWidget>
#include <QtWidgets/QMainWindow>

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(QMainWindow *parent = 0);

  virtual ~MainWindow();

private slots:

  void OnExit();

  void Play();

  void Pause();

  void Step(unsigned int steps = 1);

  void ShowSourceCode();

  void ShowWiki();

  void StepTimeMsChanged(int step_time_ms);

private:
  void OnWorldControl(const ignition::msgs::WorldControl &msg);
  void OnWorldStats(const ignition::msgs::WorldStatistics &msg);

  ignition::transport::Node node;
  Ui::MainWindow *ui;
  ignition::transport::Node::Publisher world_control_pub;
  ignition::transport::Node::Publisher physics_pub;
};
