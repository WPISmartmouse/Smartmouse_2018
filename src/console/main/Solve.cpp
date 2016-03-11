#ifdef CONSOLE

#include <errno.h>
#include <fstream>
#include "CommanDuino.h"
#include "ConsoleMaze.h"
#include "Mouse.h"
#include "ConsoleTimer.h"
#include "Flood.h"
#include "WallFollow.h"
#include "commands/SolveCommand.h"

int main(int argc, char* argv[]){

  std::string maze_file;
  if (argc < 2) {
    maze_file = "../mazes/16x16.mz";
  }
  else {
    maze_file = std::string(argv[1]);
  }

  std::fstream fs;
  fs.open(maze_file, std::fstream::in);
  Mouse mouse;

  if (fs.good()){
    ConsoleMaze maze(fs, &mouse);
    ConsoleTimer timer;
    Command::setTimerImplementation(&timer);
    Scheduler scheduler(new SolveCommand(&maze));

    while (true) {
      scheduler.run();
    }

    //maze.print_maze_mouse();
    //printf("SOLUTION: %s\n", solution);
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
