#include <QApplication>
#include <QWidget>
#include <getopt.h>
#include <QtCore/QUrl>
#include <QtGui/QDesktopServices>

#include "main.h"
#include "ui_mainwindow.h"
#include "Server.h"
#include "TopicNames.h"
#include "Time.h"

void PrintVersionInfo();

MainWindow::MainWindow(QMainWindow *parent) :
    QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  connect(ui->actionExit, &QAction::triggered, this, &MainWindow::OnExit);
  connect(ui->actionSourceCode, &QAction::triggered, this, &MainWindow::ShowSourceCode);
  connect(ui->actionWiki, &QAction::triggered, this, &MainWindow::ShowWiki);
  connect(ui->play_button, &QPushButton::clicked, this, &MainWindow::Play);
  connect(ui->pause_button, &QPushButton::clicked, this, &MainWindow::Pause);
  connect(ui->step_once_button, &QPushButton::clicked, this, &MainWindow::Step);
  connect(ui->step_time_ms_spinner, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &MainWindow::StepTimeMsChanged);

  world_control_pub = node.Advertise<ignition::msgs::WorldControl>(TopicNames::kWorldControl);
  physics_pub = node.Advertise<ignition::msgs::Physics>(TopicNames::kPhysics);
  node.Subscribe(TopicNames::kWorldControl, &MainWindow::OnWorldControl, this);
  node.Subscribe(TopicNames::kWorldStatistics, &MainWindow::OnWorldStats, this);
}

MainWindow::~MainWindow() {
  delete ui;
}

void MainWindow::OnExit() {
  ignition::msgs::WorldControl quit_msg;
  quit_msg.set_quit(true);
  world_control_pub.Publish(quit_msg);
  QApplication::quit();
}

void MainWindow::Play() {
  ignition::msgs::WorldControl play_msg;
  play_msg.set_pause(false);
  world_control_pub.Publish(play_msg);
}

void MainWindow::Pause() {
  ignition::msgs::WorldControl pause_msg;
  pause_msg.set_pause(true);
  world_control_pub.Publish(pause_msg);
}

void MainWindow::Step(unsigned int steps) {
  ignition::msgs::WorldControl step_msg;
  step_msg.set_multi_step(steps);
  world_control_pub.Publish(step_msg);
}

void MainWindow::StepTimeMsChanged(int step_time_ms) {
  ignition::msgs::Physics physics_msg;
  physics_msg.set_step_time_ms(step_time_ms);
  physics_pub.Publish(physics_msg);
}

void MainWindow::OnWorldControl(const ignition::msgs::WorldControl &msg) {
  std::cout << msg.DebugString() << std::endl;
}

void MainWindow::OnWorldStats(const ignition::msgs::WorldStatistics &msg) {
  Time time(msg.sim_time());
  ui->time->setText(QString::fromStdString(time.FormattedString()));
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
  window.show();

  int ret_code = app.exec();

  server.join();

  return ret_code;
}

void PrintVersionInfo() {
  printf("SmartmouseSim v 0.0.0\n");
}

