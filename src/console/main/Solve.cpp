#ifdef CONSOLE

#include <errno.h>
#include <fstream>
#include "CommanDuino.h"
#include "ConsoleMaze.h"
#include "ConsoleMouse.h"
#include "ConsoleTimer.h"
#include "Flood.h"
#include "WallFollow.h"
#include "commands/SolveCommand.h"

Scheduler scheduler(new SolveCommand());

ConsoleTimer timer;

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
  ConsoleMouse mouse;
  Command::setTimerImplementation(&timer);

  if (fs.good()){
    ConsoleMaze maze(fs, &mouse);
    Flood solver(&maze);
    solver.setup();
    char *solution = solver.solve();
    solver.teardown();
    maze.print_maze_mouse();
    printf("SOLUTION: %s\n", solution);
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
