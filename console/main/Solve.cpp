#ifdef CONSOLE

#include <fstream>
#include <iostream>
#include <string.h>

#include "CommanDuino.h"
#include "StepSolveCommand.h"
#include "SolveCommand.h"
#include "ConsoleMaze.h"
#include "ConsoleMouse.h"
#include "ConsoleTimer.h"
#include "Mouse.h"
#include "Flood.h"
#include "WallFollow.h"

int main(int argc, char* argv[]){

  std::string maze_file;
  bool step = false;
  if (argc <= 1) {
    maze_file = "../mazes/16x16.mz";
    printf("Using default maze ../mazes/16x16.mz\n");
  }
  if (argc == 2) {
    if (strncmp("--step", argv[1], 6) == 0) {
      step = true;
      maze_file = "../mazes/16x16.mz";
    }
    else {
      maze_file = std::string(argv[1]);
    }
  }
  else if (argc == 3) {
    maze_file = std::string(argv[1]);
    if (strncmp("--step", argv[2], 6) == 0) {
      step = true;
    }
  }

  std::fstream fs;
  fs.open(maze_file, std::fstream::in);

  if (fs.good()){
    ConsoleMaze maze(fs);
    ConsoleMouse::inst()->seedMaze(&maze);
    ConsoleTimer timer;
    Command::setTimerImplementation(&timer);

    Scheduler *scheduler;
    if (step) {
      scheduler = new Scheduler(new StepSolveCommand(new Flood(ConsoleMouse::inst())));
    }
    else {
      scheduler = new Scheduler(new SolveCommand(new Flood(ConsoleMouse::inst())));
    }

    while (!scheduler->run());

    fs.close();
    return EXIT_SUCCESS;
  }
  else {
		printf("error opening maze file!\n");
    fs.close();
    return EXIT_FAILURE;
  }
}
#endif
