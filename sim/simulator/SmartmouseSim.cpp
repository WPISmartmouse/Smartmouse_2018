#include <getopt.h>
#include <cstdio>
#include <cstdlib>
#include <lib/Server.h>
#include <QtWidgets/QApplication>
#include <lib/Client.h>

void PrintVersionInfo() {
  printf("SmartmouseSim v 0.0.0\n");
}

int main(int argc, char *argv[]) {
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

  // Start physics thread
  Server server;
  server.start();

  int return_code = 0;
  Client *window;
  do {
    QApplication app(argc, argv);
    window = new Client();
    window->setWindowTitle("Smartmouse Simulator");
    window->showMaximized();
    return_code = app.exec();
  } while (return_code == Client::kRestartCode);

  smartmouse::msgs::ServerControl quit_msg;
  quit_msg.set_quit(true);
  window->Exit();

  server.join();

  return return_code;
}
