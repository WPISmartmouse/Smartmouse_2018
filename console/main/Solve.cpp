#ifdef CONSOLE

#include <errno.h>
#include <fstream>
#include "CommanDuino.h"
#include "ConsoleMaze.h"
#include "Mouse.h"
#include "ConsoleTimer.h"
#include "ConsoleMouse.h"
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

  if (fs.good()){
    ConsoleMaze maze(fs);
    ConsoleTimer timer;
    ConsoleMouse::inst()->seedMaze(&maze);
    Command::setTimerImplementation(&timer);
    Scheduler scheduler(new SolveCommand(new WallFollow(ConsoleMouse::inst())));

    while (true) {
      scheduler.run();
    }

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
