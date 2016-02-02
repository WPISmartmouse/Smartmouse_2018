#include <fstream>
#include <string>

#include "maze.h"
#include "maze_io.h"

int main(int argc, char *argv[]){
  std::string maze_file;
  if (argc < 2) {
    maze_file = "mazes/16x16.mz";
  }
  else {
    maze_file = std::string(argv[1]);
  }

  std::fstream fs;
  fs.open(maze_file, std::fstream::in);

  if (fs.good()){
    Maze *maze = new Maze(fs);
    print_maze(maze);
    fs.close();
    return EXIT_SUCCESS;
  }
  else {
		printf("error opening maze file!\n");
    fs.close();
    return EXIT_FAILURE;
  }
}
