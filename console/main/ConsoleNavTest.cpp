#include <fstream>
#include <iostream>
#include <string.h>

#include <common/commanduino/CommanDuino.h>
#include <common/commands/SolveCommand.h>
#include "ConsoleMaze.h"
#include "ConsoleMouse.h"
#include "ConsoleTimer.h"
#include <common/Flood.h>
#include <common/WallFollow.h>
#include <common/commands/NavTestCommand.h>

int main(int argc, char *argv[]) {

  std::string maze_file;
  bool step = false;
  if (argc <= 1) {
    maze_file = "../mazes/16x16.mz";
    printf("Using default maze ../mazes/16x16.mz\n");
  }
  if (argc == 2) {
      maze_file = std::string(argv[1]);
  }

  std::fstream fs;
  fs.open(maze_file, std::fstream::in);

  if (fs.good()) {
    ConsoleMaze maze(fs);
    ConsoleMouse::inst()->seedMaze(&maze);
    ConsoleTimer timer;
    Command::setTimerImplementation(&timer);

    Scheduler *scheduler;
    scheduler = new Scheduler(new NavTestCommand(new Flood(ConsoleMouse::inst())));

    while (!scheduler->run());

    fs.close();
    return EXIT_SUCCESS;
  } else {
    printf("error opening maze file!\n");
    fs.close();
    return EXIT_FAILURE;
  }
}

