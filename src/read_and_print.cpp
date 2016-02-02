#include <fstream>
#include <string>

#include "maze.h"
#include "maze_io.h"

int main(int argc, char *argv[]){
  std::string maze_file(argv[1]);
  if (argc < 2) {
    maze_file = "mazes/16x16.mz";
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
    fs.close();
    return EXIT_FAILURE;
  }
}
