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

  RestoreSettings();

  server_control_pub_ = node_.Advertise<smartmouse::msgs::ServerControl>(TopicNames::kWorldControl);
  physics_pub_ = node_.Advertise<smartmouse::msgs::PhysicsConfig>(TopicNames::kPhysics);
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

void Client::TimePerStepMsChanged(int step_time_ms) {
  smartmouse::msgs::PhysicsConfig physics_msg;
  physics_msg.set_ns_of_sim_per_step(step_time_ms * 1000000u);
  physics_pub_.Publish(physics_msg);
}

void Client::OnWorldStats(const smartmouse::msgs::WorldStatistics &msg) {
  Time time(msg.sim_time());
  emit SetRealTime(QString::number(msg.real_time_factor(), 'f', 4));
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

void Client::LoadNewMaze() {
  QString file_name = QFileDialog::getOpenFileName(this, tr("Open Maze"), maze_files_dir_, tr("Maze Files (*.mz)"));

  if (file_name != nullptr) {
    QFileInfo file_info(file_name);
    maze_files_dir_ = file_info.dir().absolutePath();
    default_maze_file_name_ = file_name;
    settings_->setValue("gui/default_maze_file_name", default_maze_file_name_);
    settings_->setValue("gui/maze_files_directory", maze_files_dir_);

    // TODO: Actually load the maze here

    ui_->maze_file_name_label->setText(file_info.fileName());
  }
}

void Client::LoadDefaultMaze() {
  if (default_maze_file_name_ != nullptr) {
    QFileInfo file_info(default_maze_file_name_);

    // TODO: Actually load the maze here

    ui_->maze_file_name_label->setText(file_info.fileName());
  }
}

void Client::ConfigureGui() {
  maze_widget_ = new MazeWidget();
  ui_->gui_tabs->addTab(maze_widget_, maze_widget_->getTabName());
  ui_->main_tab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Expanding);
  ui_->main_tab->setMaximumWidth(300);

  connect(ui_->load_maze_button, &QPushButton::clicked, this, &Client::LoadNewMaze);
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
          &Client::TimePerStepMsChanged);
  QObject::connect(this, &Client::SetRealTime, ui_->real_time_value_label, &QLabel::setText);
  QObject::connect(this, &Client::SetTime, ui_->time_value_label, &QLabel::setText);

  QFile styleFile(":/style.qss");
  styleFile.open(QFile::ReadOnly);
  QString style(styleFile.readAll());
  this->setStyleSheet(style);
}

void Client::closeEvent(QCloseEvent *event) {
  writeSettings();
  event->accept();
}

void Client::writeSettings() {
  settings_->setValue("gui/tab_splitter", ui_->tab_splitter->saveState());
  settings_->setValue("gui/info_tabs", ui_->info_tabs->currentIndex());
}

void Client::RestoreSettings() {
  QCoreApplication::setOrganizationName("WPISmartmouse");
  QCoreApplication::setOrganizationDomain("smartmouse.com");
  QCoreApplication::setApplicationName("Smartmouse Sim");
  settings_ = new QSettings();

  const QByteArray splitter_state = settings_->value("gui/tab_splitter").toByteArray();
  if (!splitter_state.isEmpty()) {
    ui_->tab_splitter->restoreState(splitter_state);
  }

  const int info_tab_index = settings_->value("gui/info_tabs").toInt();
  ui_->info_tabs->setCurrentIndex(info_tab_index);

  maze_files_dir_ = settings_->value("gui/maze_files_directory").toString();
  default_maze_file_name_ = settings_->value("gui/default_maze_file_name").toString();
  LoadDefaultMaze();
}
