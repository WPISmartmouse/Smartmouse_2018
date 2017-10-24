#include <getopt.h>
#include <cstdio>
#include <cstdlib>
#include <lib/Server.h>
#include <QtWidgets/QApplication>
#include <lib/Client.h>
#include <lib/common/TopicNames.h>

void PrintVersionInfo() {
  printf("SmartmouseSim v 0.0.0\n");
}

int main(int argc, char *argv[]) {

  // TODO: add proper argument parsing, and a way to pass them to the Client
  int c;

  while (1) {
    c = getopt_long(argc, argv, "-v", nullptr, nullptr);

    /* Detect the end of the options. */
    if (c == -1)
      break;

    switch (c) {
      case 'v': {
        PrintVersionInfo();
        return EXIT_SUCCESS;
      }
      case '?': { break; }
      default: {
        std::cout << "Invalid Arugments" << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  ignition::transport::Node node;
  auto server_pub = node.Advertise<smartmouse::msgs::ServerControl>(TopicNames::kServerControl);

  int return_code = 0;
  Client *window;
  do {
    // Start physics thread
    Server server;
    server.Start();

    QApplication app(argc, argv);
    window = new Client();
    window->setWindowTitle("Smartmouse Simulator");
    window->showMaximized();
    return_code = app.exec();

    smartmouse::msgs::ServerControl quit_msg;
    quit_msg.set_quit(true);
    server_pub.Publish(quit_msg);
    server.Join();
  } while (return_code == Client::kRestartCode);

  QApplication::quit();

  return return_code;
}
