#include <getopt.h>

#include <QApplication>
#include <QWidget>
#include <QtCore/QUrl>
#include <QtGui/QDesktopServices>
#include <QtWidgets/QAction>
#include <QtWidgets/QSpinBox>

#include <sim/simulator/lib/Server.h>
#include <sim/simulator/lib/TopicNames.h>
#include <sim/simulator/main.h>

#include "ui_mainwindow.h"

void PrintVersionInfo();

MainWindow::MainWindow(QMainWindow *parent) :
    QMainWindow(parent), ui_(new Ui::MainWindow) {
  ui_->setupUi(this);

  world_widget_ = new WorldWidget();
  ui_->right_side_layout->insertWidget(0, world_widget_);

  connect(ui_->actionExit, &QAction::triggered, this, &MainWindow::OnExit);
  connect(ui_->actionSourceCode, &QAction::triggered, this, &MainWindow::ShowSourceCode);
  connect(ui_->actionWiki, &QAction::triggered, this, &MainWindow::ShowWiki);
  connect(ui_->play_button, &QPushButton::clicked, this, &MainWindow::Play);
  connect(ui_->pause_button, &QPushButton::clicked, this, &MainWindow::Pause);
  connect(ui_->step_button, &QPushButton::clicked, this, &MainWindow::Step);
  connect(ui_->real_time_factor_spinner,
          static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          this,
          &MainWindow::RealTimeFactorChanged);
  connect(ui_->step_spinner,
          static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          this,
          &MainWindow::StepCountChanged);
  connect(ui_->ms_per_step_spinner,
          static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          this,
          &MainWindow::StepTimeMsChanged);

  server_control_pub_ = node_.Advertise<smartmouse::msgs::ServerControl>(TopicNames::kWorldControl);
  physics_pub_ = node_.Advertise<smartmouse::msgs::PhysicsConfig>(TopicNames::kPhysics);
  node_.Subscribe(TopicNames::kWorldControl, &MainWindow::OnWorldControl, this);
  node_.Subscribe(TopicNames::kWorldStatistics, &MainWindow::OnWorldStats, this);
  node_.Subscribe(TopicNames::kGuiActions, &MainWindow::OnGuiActions, this);
  node_.Subscribe(TopicNames::kPhysics, &MainWindow::OnPhysics, this);

  // publish the initial configuration
  smartmouse::msgs::PhysicsConfig initial_physics_config;
  initial_physics_config.set_ns_of_sim_per_step(1000000u); // 1ms
  initial_physics_config.set_real_time_factor(0.5);
  physics_pub_.Publish(initial_physics_config);

  smartmouse::msgs::ServerControl initial_server_control;
  initial_server_control.set_pause(true);
  server_control_pub_.Publish(initial_server_control);
}

MainWindow::~MainWindow() {
  delete ui_;
}

void MainWindow::OnExit() {
  smartmouse::msgs::ServerControl quit_msg;
  quit_msg.set_quit(true);
  server_control_pub_.Publish(quit_msg);
  QApplication::quit();
}

void MainWindow::Play() {
  smartmouse::msgs::ServerControl play_msg;
  play_msg.set_pause(false);
  server_control_pub_.Publish(play_msg);
}

void MainWindow::Pause() {
  smartmouse::msgs::ServerControl pause_msg;
  pause_msg.set_pause(true);
  server_control_pub_.Publish(pause_msg);
}

void MainWindow::Step() {
  smartmouse::msgs::ServerControl step_msg;
  step_msg.set_step(step_count_);
  server_control_pub_.Publish(step_msg);
}

void MainWindow::RealTimeFactorChanged(double real_time_factor) {
  smartmouse::msgs::PhysicsConfig rtf_msg;
  rtf_msg.set_real_time_factor(real_time_factor);
  physics_pub_.Publish(rtf_msg);
}

void MainWindow::StepCountChanged(int step_time_ms) {
  if (step_time_ms > 0) {
    step_count_ = (unsigned int) step_time_ms;
  }
}

void MainWindow::StepTimeMsChanged(int step_time_ms) {
  smartmouse::msgs::PhysicsConfig physics_msg;
  physics_msg.set_ns_of_sim_per_step(step_time_ms * 1000000u);
  physics_pub_.Publish(physics_msg);
}

void MainWindow::OnWorldControl(const smartmouse::msgs::ServerControl &msg) {
  std::cout << msg.DebugString() << std::endl;
}

void MainWindow::OnWorldStats(const smartmouse::msgs::WorldStatistics &msg) {
  Time time(msg.sim_time());
  ui_->time_value_label->setText(QString::fromStdString(time.FormattedString()));
  ui_->real_time_value_label->setText(QString::number(msg.real_time_factor()));
}

void MainWindow::OnPhysics(const smartmouse::msgs::PhysicsConfig &msg) {
  if (msg.has_ns_of_sim_per_step()) {
    ui_->ms_per_step_spinner->setValue(msg.ns_of_sim_per_step() / 1000000);
  }
  if (msg.has_real_time_factor()) {
    ui_->real_time_factor_spinner->setValue(msg.real_time_factor());
  }
}

void MainWindow::OnGuiActions(const smartmouse::msgs::GuiActions &msg) {
  if (msg.source_code_action()) {
    ShowSourceCode();
  }
}

void MainWindow::ShowWiki() {
  QDesktopServices::openUrl(QUrl("https://github.com/WPISmartMouse/SmartmouseSim/wiki", QUrl::TolerantMode));
}

void MainWindow::ShowSourceCode() {
  QDesktopServices::openUrl(QUrl("https://github.com/WPISmartMouse/SmartmouseSim", QUrl::TolerantMode));
}

int main(int argc, char *argv[]) {
  int c;

  while (1) {
    c = getopt_long(argc, argv, "-v", nullptr, nullptr);

    /* Detect the end of the options. */
    if (c == -1)
      break;

    switch (c) {
      case 'v':PrintVersionInfo();
        return EXIT_SUCCESS;
      case '?':break;
      default:abort();
    }
  }

  // Start physics thread
  Server server;
  server.start();

  QApplication app(argc, argv);

  MainWindow window;

  window.setWindowTitle("SmartMouse Simulator");
  window.showMaximized();

  int ret_code = app.exec();

  smartmouse::msgs::ServerControl quit_msg;
  quit_msg.set_quit(true);
  window.OnExit();

  server.join();

  return ret_code;
}

void PrintVersionInfo() {
  printf("SmartmouseSim v 0.0.0\n");
}

