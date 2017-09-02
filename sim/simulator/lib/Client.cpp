#include <QtCore/QUrl>
#include <QtGui/QDesktopServices>
#include <QtWidgets/QAction>
#include <QtWidgets/QSpinBox>

#include <sim/simulator/lib/Server.h>
#include <sim/simulator/lib/Client.h>
#include <sim/simulator/lib/TopicNames.h>

#include "ui_mainwindow.h"

Client::Client(QMainWindow *parent) :
    QMainWindow(parent), ui_(new Ui::MainWindow) {
  ui_->setupUi(this);

  ConfigureGui();

  server_control_pub_ = node_.Advertise<smartmouse::msgs::ServerControl>(TopicNames::kWorldControl);
  physics_pub_ = node_.Advertise<smartmouse::msgs::PhysicsConfig>(TopicNames::kPhysics);
  node_.Subscribe(TopicNames::kWorldControl, &Client::OnWorldControl, this);
  node_.Subscribe(TopicNames::kWorldStatistics, &Client::OnWorldStats, this);
  node_.Subscribe(TopicNames::kGuiActions, &Client::OnGuiActions, this);
  node_.Subscribe(TopicNames::kPhysics, &Client::OnPhysics, this);

  // publish the initial configuration
  smartmouse::msgs::PhysicsConfig initial_physics_config;
  initial_physics_config.set_ns_of_sim_per_step(1000000u); // 1ms
  initial_physics_config.set_real_time_factor(1);
  physics_pub_.Publish(initial_physics_config);

  smartmouse::msgs::ServerControl initial_server_control;
  initial_server_control.set_pause(true);
  server_control_pub_.Publish(initial_server_control);
}

Client::~Client() {
  delete ui_;
}

void Client::OnExit() {
  smartmouse::msgs::ServerControl quit_msg;
  quit_msg.set_quit(true);
  server_control_pub_.Publish(quit_msg);
  QApplication::quit();
}

void Client::Play() {
  smartmouse::msgs::ServerControl play_msg;
  play_msg.set_pause(false);
  server_control_pub_.Publish(play_msg);
}

void Client::Pause() {
  smartmouse::msgs::ServerControl pause_msg;
  pause_msg.set_pause(true);
  server_control_pub_.Publish(pause_msg);
}

void Client::Step() {
  smartmouse::msgs::ServerControl step_msg;
  step_msg.set_step(step_count_);
  server_control_pub_.Publish(step_msg);
}

void Client::RealTimeFactorChanged(double real_time_factor) {
  smartmouse::msgs::PhysicsConfig rtf_msg;
  rtf_msg.set_real_time_factor(real_time_factor);
  physics_pub_.Publish(rtf_msg);
}

void Client::StepCountChanged(int step_time_ms) {
  if (step_time_ms > 0) {
    step_count_ = (unsigned int) step_time_ms;
  }
}

void Client::StepTimeMsChanged(int step_time_ms) {
  smartmouse::msgs::PhysicsConfig physics_msg;
  physics_msg.set_ns_of_sim_per_step(step_time_ms * 1000000u);
  physics_pub_.Publish(physics_msg);
}

void Client::OnWorldControl(const smartmouse::msgs::ServerControl &msg) {
  std::cout << msg.DebugString() << std::endl;
}

void Client::OnWorldStats(const smartmouse::msgs::WorldStatistics &msg) {
  Time time(msg.sim_time());
  emit SetRealTime(QString::number(msg.real_time_factor()));
  emit SetTime(QString::fromStdString(time.FormattedString()));
}

void Client::OnPhysics(const smartmouse::msgs::PhysicsConfig &msg) {
  if (msg.has_ns_of_sim_per_step()) {
    ui_->ms_per_step_spinner->setValue(msg.ns_of_sim_per_step() / 1000000);
  }
  if (msg.has_real_time_factor()) {
    ui_->real_time_factor_spinner->setValue(msg.real_time_factor());
  }
}

void Client::OnGuiActions(const smartmouse::msgs::GuiActions &msg) {
  if (msg.source_code_action()) {
    ShowSourceCode();
  }
}

void Client::ShowWiki() {
  QDesktopServices::openUrl(QUrl("https://github.com/WPISmartMouse/SmartmouseSim/wiki", QUrl::TolerantMode));
}

void Client::ShowSourceCode() {
  QDesktopServices::openUrl(QUrl("https://github.com/WPISmartMouse/SmartmouseSim", QUrl::TolerantMode));
}

void Client::ConfigureGui() {
  world_widget_ = new WorldWidget();
  ui_->right_side_layout->insertWidget(0, world_widget_);

  connect(ui_->actionExit, &QAction::triggered, this, &Client::OnExit);
  connect(ui_->actionSourceCode, &QAction::triggered, this, &Client::ShowSourceCode);
  connect(ui_->actionWiki, &QAction::triggered, this, &Client::ShowWiki);
  connect(ui_->play_button, &QPushButton::clicked, this, &Client::Play);
  connect(ui_->pause_button, &QPushButton::clicked, this, &Client::Pause);
  connect(ui_->step_button, &QPushButton::clicked, this, &Client::Step);
  // Casting is to handle overloaded slot valueChanged. Don't overload slots!
  connect(ui_->real_time_factor_spinner,
          static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          this,
          &Client::RealTimeFactorChanged);
  connect(ui_->step_spinner,
          static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          this,
          &Client::StepCountChanged);
  connect(ui_->ms_per_step_spinner,
          static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          this,
          &Client::StepTimeMsChanged);
  QObject::connect(this, &Client::SetRealTime, ui_->real_time_value_label, &QLabel::setText);
  QObject::connect(this, &Client::SetTime, ui_->time_value_label, &QLabel::setText);

  ui_->tab_splitter->setHandleWidth(10);
}
