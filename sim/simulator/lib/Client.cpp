#include <fstream>
#include <QtCore/QUrl>
#include <QtGui/QDesktopServices>
#include <QtWidgets/QAction>
#include <QtWidgets/QSpinBox>

#include <lib/common/TopicNames.h>
#include <sim/simulator/lib/Server.h>
#include <sim/simulator/lib/Client.h>
#include <sim/simulator/msgs/pid_constants.pb.h>

#include "ui_mainwindow.h"

Client::Client(QMainWindow *parent) :
    QMainWindow(parent), ui_(new Ui::MainWindow) {
  ui_->setupUi(this);

  uuid = sole::uuid1();

  server_control_pub_ = node_.Advertise<smartmouse::msgs::ServerControl>(TopicNames::kServerControl);
  physics_pub_ = node_.Advertise<smartmouse::msgs::PhysicsConfig>(TopicNames::kPhysics);
  maze_pub_ = node_.Advertise<smartmouse::msgs::Maze>(TopicNames::kMaze);
  robot_description_pub_ = node_.Advertise<smartmouse::msgs::RobotDescription>(TopicNames::kRobotDescription);
  robot_command_pub_ = node_.Advertise<smartmouse::msgs::RobotCommand>(TopicNames::kRobotCommand);
  pid_setpoints_pub_ = node_.Advertise<ignition::msgs::Vector2d>(TopicNames::kPIDSetpoints);
  pid_constants_pub_ = node_.Advertise<smartmouse::msgs::PIDConstants>(TopicNames::kPIDConstants);
  node_.Subscribe(TopicNames::kWorldStatistics, &Client::OnWorldStats, this);
  node_.Subscribe(TopicNames::kGuiActions, &Client::OnGuiActions, this);
  node_.Subscribe(TopicNames::kPhysics, &Client::OnPhysics, this);
  node_.Subscribe(TopicNames::kServerControl, &Client::OnServerControl, this);

  ConfigureGui();
  RestoreSettings();

  // publish the initial configuration
  smartmouse::msgs::PhysicsConfig initial_physics_config;
  initial_physics_config.set_ns_of_sim_per_step(1000000u);
  initial_physics_config.set_author(uuid.str());
  physics_pub_.Publish(initial_physics_config);

  // publish initial config of the server
  smartmouse::msgs::ServerControl initial_server_control;
  initial_server_control.set_author(uuid.str());
  initial_server_control.set_pause(false);
  initial_server_control.set_reset_robot(true);
  initial_server_control.set_reset_time(true);
  server_control_pub_.Publish(initial_server_control);
}

void Client::Exit() {
  SaveSettings();
  smartmouse::msgs::ServerControl quit_msg;
  quit_msg.set_quit(true);
  quit_msg.set_author(uuid.str());
  server_control_pub_.Publish(quit_msg);
  QApplication::exit(0);
}

void Client::Restart() {
  SaveSettings();
  QApplication::exit(kRestartCode);
}

void Client::TogglePlayPause() {
  smartmouse::msgs::ServerControl msg;
  msg.set_toggle_play_pause(true);
  msg.set_author(uuid.str());
  server_control_pub_.Publish(msg);
}

void Client::SetStatic() {
  smartmouse::msgs::ServerControl static_msg;
  static_msg.set_author(uuid.str());
  static_msg.set_static_(ui_->static_checkbox->isChecked());
  server_control_pub_.Publish(static_msg);
}

void Client::Step() {
  smartmouse::msgs::ServerControl step_msg;
  step_msg.set_author(uuid.str());
  step_msg.set_step(step_count_);
  server_control_pub_.Publish(step_msg);
}

void Client::ResetMouse() {
  smartmouse::msgs::ServerControl reset_msg;
  reset_msg.set_author(uuid.str());
  reset_msg.set_reset_robot(true);
  server_control_pub_.Publish(reset_msg);
}

void Client::ResetTime() {
  smartmouse::msgs::ServerControl reset_msg;
  reset_msg.set_author(uuid.str());
  reset_msg.set_reset_time(true);
  server_control_pub_.Publish(reset_msg);
}

void Client::RealTimeFactorChanged(double real_time_factor) {
  smartmouse::msgs::PhysicsConfig rtf_msg;
  rtf_msg.set_real_time_factor(real_time_factor);
  rtf_msg.set_author(uuid.str());
  physics_pub_.Publish(rtf_msg);
}

void Client::StepCountChanged(int step_time_ms) {
  if (step_time_ms > 0) {
    step_count_ = (unsigned int) step_time_ms;
  }
}

void Client::TimePerStepMsChanged(int step_time_ms) {
  smartmouse::msgs::PhysicsConfig time_per_step_msg;
  time_per_step_msg.set_ns_of_sim_per_step(step_time_ms * 1000000u);
  time_per_step_msg.set_author(uuid.str());
  physics_pub_.Publish(time_per_step_msg);
}

void Client::OnWorldStats(const smartmouse::msgs::WorldStatistics &msg) {
  Time time(msg.sim_time());
  emit SetRealTime(QString::number(msg.real_time_factor(), 'f', 4));
  emit SetTime(QString::fromStdString(time.FormattedString()));
}

void Client::OnPhysics(const smartmouse::msgs::PhysicsConfig &msg) {
  if (msg.author() == uuid.str()) {
    return;
  }
  if (msg.has_ns_of_sim_per_step()) {
    ui_->ms_per_step_spinner->setValue(msg.ns_of_sim_per_step() / 1000000);
  }
  if (msg.has_real_time_factor()) {
    ui_->real_time_factor_spinner->setValue(msg.real_time_factor());
  }
}

void Client::OnServerControl(const smartmouse::msgs::ServerControl &msg) {
  if (msg.has_pause()) {
    if (msg.pause()) {
      ui_->play_button->setText("Play");
    } else {
      ui_->play_button->setText("Pause");
    }
  }
}

void Client::OnGuiActions(const smartmouse::msgs::GuiActions &msg) {
  if (msg.source_code_action()) {
    ShowSourceCode();
  }
}

void Client::ShowWiki() {
  QDesktopServices::openUrl(QUrl("https://github.com/WPISmartMouse/Smartmouse_2018/wiki", QUrl::TolerantMode));
}

void Client::ShowSourceCode() {
  QDesktopServices::openUrl(QUrl("https://github.com/WPISmartMouse/Smartmouse_2018", QUrl::TolerantMode));
}

void Client::ShowKeyboardShortcuts() {
  QDesktopServices::openUrl(QUrl("https://github.com/WPISmartMouse/Smartmouse_2018/wiki/Simulator-Hotkeys",
                                 QUrl::TolerantMode));
}

void Client::LoadNewMouse() {
  QString file_name = QFileDialog::getOpenFileName(this, tr("Open Mouse"), mouse_files_dir_, tr("Mouse Files (*.ms)"));

  if (!file_name.isEmpty()) {
    QFileInfo file_info(file_name);
    mouse_files_dir_ = file_info.dir().absolutePath();
    default_mouse_file_name_ = file_name;
    settings_->setValue("gui/default_mouse_file_name", default_mouse_file_name_);
    settings_->setValue("gui/mouse_files_directory", mouse_files_dir_);

    std::ifstream fs;
    fs.open(file_info.absoluteFilePath().toStdString(), std::fstream::in);

    smartmouse::msgs::RobotDescription robot_description_msg = smartmouse::msgs::Convert(fs);
    robot_description_pub_.Publish(robot_description_msg);
    ui_->mouse_file_name_label->setText(file_info.fileName());
  }
}

void Client::LoadNewMaze() {
  QString file_name = QFileDialog::getOpenFileName(this, tr("Open Maze"), maze_files_dir_, tr("Maze Files (*.mz)"));

  if (!file_name.isEmpty()) {
    QFileInfo file_info(file_name);
    maze_files_dir_ = file_info.dir().absolutePath();
    default_maze_file_name_ = file_name;
    settings_->setValue("gui/default_maze_file_name", default_maze_file_name_);
    settings_->setValue("gui/maze_files_directory", maze_files_dir_);

    std::ifstream fs;
    fs.open(file_info.absoluteFilePath().toStdString(), std::fstream::in);
    AbstractMaze maze(fs);
    smartmouse::msgs::Maze maze_msg = smartmouse::msgs::Convert(&maze);
    maze_pub_.Publish(maze_msg);
    ui_->maze_file_name_label->setText(file_info.fileName());
  }
}

void Client::LoadRandomMaze() {
  AbstractMaze maze = AbstractMaze::gen_random_legal_maze();
  smartmouse::msgs::Maze maze_msg = smartmouse::msgs::Convert(&maze);
  maze_pub_.Publish(maze_msg);
}

void Client::LoadDefaultMouse() {
  if (!default_mouse_file_name_.isEmpty()) {
    QFileInfo file_info(default_mouse_file_name_);

    std::ifstream fs;
    std::string mouse_filename = file_info.absoluteFilePath().toStdString();
    fs.open(mouse_filename, std::fstream::in);
    if (fs.good()) {
      smartmouse::msgs::RobotDescription mouse_msg = smartmouse::msgs::Convert(fs);
      robot_description_pub_.Publish(mouse_msg);
      ui_->mouse_file_name_label->setText(file_info.fileName());
    } else {
      std::cout << "default mouse file [" << mouse_filename << "] not found\n";
    }
  } else {
    std::cout << "no default mouse\n";
    // TODO: handle this case
  }
}

void Client::LoadDefaultMaze() {
  if (!default_maze_file_name_.isEmpty()) {
    QFileInfo file_info(default_maze_file_name_);

    std::ifstream fs;
    std::string maze_filename = file_info.absoluteFilePath().toStdString();
    fs.open(maze_filename, std::fstream::in);
    if (fs.good()) {
      AbstractMaze maze(fs);
      smartmouse::msgs::Maze maze_msg = smartmouse::msgs::Convert(&maze);
      maze_pub_.Publish(maze_msg);
      ui_->maze_file_name_label->setText(file_info.fileName());
    } else {
      std::cout << "default mouse file [" << maze_filename << "] not found. Loading random maze.\n";
      LoadRandomMaze();
    }
  } else {
    std::cout << "No default maze. Loading random maze\n";
    LoadRandomMaze();
  }
}

void Client::SendRobotCmd() {
  smartmouse::msgs::RobotCommand cmd;
  auto left = cmd.mutable_left();
  auto right = cmd.mutable_right();
  left->set_abstract_force(ui_->left_f_spinbox->value());
  right->set_abstract_force(ui_->right_f_spinbox->value());
  robot_command_pub_.Publish(cmd);
}

void Client::SendTeleportCmd() {
  smartmouse::msgs::ServerControl cmd;
  cmd.set_reset_robot(true);
  cmd.set_reset_col(ui_->teleport_column_spinbox->value());
  cmd.set_reset_row(ui_->teleport_row_spinbox->value());
  cmd.set_reset_yaw(ui_->teleport_yaw_spinbox->value());
  cmd.set_author(uuid.str());
  server_control_pub_.Publish(cmd);
}

void Client::PublishPIDConstants() {
  smartmouse::msgs::PIDConstants msg;
  msg.set_kp(ui_->kp_spinbox->value());
  msg.set_ki(ui_->ki_spinbox->value());
  msg.set_kd(ui_->kd_spinbox->value());
  msg.set_kffoffset(ui_->kff_offset_spinbox->value());
  msg.set_kffscale(ui_->kff_scale_spinbox->value());
  pid_constants_pub_.Publish(msg);
}

void Client::PublishPIDSetpoints() {
  ignition::msgs::Vector2d msg;
  msg.set_x(ui_->left_setpoint_spinbox->value());
  msg.set_y(ui_->right_setpoint_spinbox->value());
  pid_setpoints_pub_.Publish(msg);
}

void Client::ConfigureGui() {
  maze_widget_ = new MazeWidget();
  ui_->gui_tabs->addTab(maze_widget_, maze_widget_->GetTabName());
  state_widget_ = new StateWidget();
  ui_->main_splitter->addWidget(state_widget_);
  ui_->info_tabs->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Expanding);
  ui_->info_tabs->setMaximumWidth(300);

  shortcut = new QShortcut(QKeySequence(tr("Space", "Toggle Play/Pause")), this);
  connect(shortcut, &QShortcut::activated, this, &Client::TogglePlayPause);

  connect(ui_->load_maze_button, &QPushButton::clicked, this, &Client::LoadNewMaze);
  connect(ui_->load_mouse_button, &QPushButton::clicked, this, &Client::LoadNewMouse);
  connect(ui_->random_maze_button, &QPushButton::clicked, this, &Client::LoadRandomMaze);
  connect(ui_->refresh_mouse_button, &QPushButton::clicked, this, &Client::LoadDefaultMouse);
  connect(ui_->actionExit, &QAction::triggered, this, &Client::Exit);
  connect(ui_->actionRestart, &QAction::triggered, this, &Client::Restart);
  connect(ui_->actionReset_Mouse, &QAction::triggered, this, &Client::ResetMouse);
  connect(ui_->actionReset_Time, &QAction::triggered, this, &Client::ResetTime);
  connect(ui_->actionSourceCode, &QAction::triggered, this, &Client::ShowSourceCode);
  connect(ui_->actionWiki, &QAction::triggered, this, &Client::ShowWiki);
  connect(ui_->actionKeyboard_Shortcuts, &QAction::triggered, this, &Client::ShowKeyboardShortcuts);
  connect(ui_->play_button, &QPushButton::clicked, this, &Client::TogglePlayPause);
  connect(ui_->step_button, &QPushButton::clicked, this, &Client::Step);
  // Casting is to handle overloaded slot valueChanged. Don't overload slots!
  connect(ui_->static_checkbox, &QCheckBox::stateChanged, this, &Client::SetStatic);
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
  connect(ui_->send_command_button, &QPushButton::clicked, this, &Client::SendRobotCmd);
  connect(ui_->teleport_button, &QPushButton::clicked, this, &Client::SendTeleportCmd);
  connect(ui_->publish_constants_button, &QPushButton::clicked, this, &Client::PublishPIDConstants);
  connect(ui_->publish_setpoints_button, &QPushButton::clicked, this, &Client::PublishPIDSetpoints);

  QFile styleFile(":/style.qss");
  styleFile.open(QFile::ReadOnly);
  QString style(styleFile.readAll());
  this->setStyleSheet(style);
}

void Client::closeEvent(QCloseEvent *event) {
  SaveSettings();
  event->accept();
}

void Client::SaveSettings() {
  settings_->setValue("gui/main_splitter", ui_->main_splitter->saveState());
  settings_->setValue("gui/info_tabs", ui_->info_tabs->currentIndex());
  settings_->setValue("gui/static_", ui_->static_checkbox->isChecked());
  settings_->setValue("gui/real_time_value", ui_->real_time_factor_spinner->value());
  settings_->setValue("gui/kp", ui_->kp_spinbox->value());
  settings_->setValue("gui/ki", ui_->ki_spinbox->value());
  settings_->setValue("gui/kd", ui_->kd_spinbox->value());
  settings_->setValue("gui/kff_offset", ui_->kff_offset_spinbox->value());
  settings_->setValue("gui/kff_scale", ui_->kff_scale_spinbox->value());
}

void Client::RestoreSettings() {
  QCoreApplication::setOrganizationName("WPISmartmouse");
  QCoreApplication::setOrganizationDomain("smartmouse.com");
  QCoreApplication::setApplicationName("SmartmouseSim");
  settings_ = new QSettings();

  const QByteArray splitter_state = settings_->value("gui/main_splitter").toByteArray();
  if (!splitter_state.isEmpty()) {
    ui_->main_splitter->restoreState(splitter_state);
  }

  const int info_tab_index = settings_->value("gui/info_tabs").toInt();
  ui_->info_tabs->setCurrentIndex(info_tab_index);

  maze_files_dir_ = settings_->value("gui/maze_files_directory").toString();
  default_maze_file_name_ = settings_->value("gui/default_maze_file_name").toString();
  LoadDefaultMaze();

  mouse_files_dir_ = settings_->value("gui/mouse_files_directory").toString();
  default_mouse_file_name_ = settings_->value("gui/default_mouse_file_name").toString();
  LoadDefaultMouse();

  ui_->static_checkbox->setChecked(settings_->value("gui/static_").toBool());

  ui_->real_time_factor_spinner->setValue(settings_->value("gui/real_time_value").toDouble());
  ui_->kp_spinbox->setValue(settings_->value("gui/kp").toDouble());
  ui_->ki_spinbox->setValue(settings_->value("gui/ki").toDouble());
  ui_->kd_spinbox->setValue(settings_->value("gui/kd").toDouble());
  ui_->kff_offset_spinbox->setValue(settings_->value("gui/kff_offset").toDouble());
  ui_->kff_scale_spinbox->setValue(settings_->value("gui/kff_scale").toDouble());
}
