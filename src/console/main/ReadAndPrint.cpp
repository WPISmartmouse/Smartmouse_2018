#ifdef CONSOLE

#include <fstream>
#include <string>
#include "Mouse.h"

#include "ConsoleMaze.h"

int main(int argc, char *argv[]){
  std::string maze_file;
  if (argc < 2) {
    maze_file = "mazes/16x16.mz";
  }
  else {
    maze_file = std::string(argv[1]);
  }

  std::fstream fs;
  Mouse mouse;
  fs.open(maze_file, std::fstream::in);

  if (fs.good()){
    ConsoleMaze maze(fs, &mouse);
    maze.print_maze();
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
