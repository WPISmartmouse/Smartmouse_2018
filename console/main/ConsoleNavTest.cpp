#include <fstream>
#include <iostream>
#include <string.h>

#include <common/commanduino/CommanDuino.h>
#include <common/commands/SolveCommand.h>
#include <console/ConsoleMouse.h>
#include <console/ConsoleTimer.h>
#include <common/core/Flood.h>
#include <common/core/WallFollow.h>
#include <common/commands/NavTestCommand.h>

int main(int argc, char *argv[]) {

  std::string maze_file;
  if (argc <= 1) {
    maze_file = "../mazes/16x16.mz";
    printf("Using default maze ../mazes/16x16.mz\n");
  }
  if (argc == 2) {
      maze_file = std::string(argv[1]);
  }

  std::ifstream fs;
  fs.open(maze_file, std::ifstream::in);

  if (fs.good()) {
    AbstractMaze maze(fs);
    ConsoleMouse::inst()->seedMaze(&maze);
    ConsoleTimer timer;
    Command::setTimerImplementation(&timer);

    Scheduler *scheduler;
    scheduler = new Scheduler(new SolveCommand(new Flood(ConsoleMouse::inst())));

    while (!scheduler->run());

    fs.close();
    return EXIT_SUCCESS;
  } else {
    printf("error opening maze file!\n");
    fs.close();
    return EXIT_FAILURE;
  }
}

