#include <fstream>
#include <iostream>

#include <common/commanduino/CommanDuino.h>
#include <common/commands/SolveCommand.h>
#include "ConsoleMaze.h"
#include "ConsoleMouse.h"
#include "ConsoleTimer.h"
#include <common/Flood.h>

int main(int argc, char *argv[]) {

  std::string maze_file;
  bool step = false;
  bool rand = false;
  if (argc <= 1) {
    rand = true;
    printf("Using random maze\n");
  }
  if (argc == 2) {
    maze_file = std::string(argv[1]);
  }

  std::fstream fs;
  fs.open(maze_file, std::fstream::in);

  if (rand) {
    AbstractMaze maze = AbstractMaze::gen_random_legal_maze();
    ConsoleMouse::inst()->seedMaze(&maze);
  } else {
    if (fs.good()) {
      ConsoleMaze m(fs);
      ConsoleMouse::inst()->seedMaze(&m);
    } else {
      printf("error opening maze file!\n");
      fs.close();
      return EXIT_FAILURE;
    }
  }
  ConsoleTimer timer;
  Command::setTimerImplementation(&timer);

  Scheduler *scheduler;
  scheduler = new Scheduler(new SolveCommand(new Flood(ConsoleMouse::inst())));

  while (!scheduler->run());

  return EXIT_SUCCESS;
}

