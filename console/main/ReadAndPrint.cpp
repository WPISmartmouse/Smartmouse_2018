#include <fstream>

#include <console/ConsoleMouse.h>

int main(int argc, char *argv[]) {
  std::string maze_file;
  if (argc < 2) {
    maze_file = "mazes/16x16.mz";
  } else {
    maze_file = std::string(argv[1]);
  }

  std::ifstream fs;
  fs.open(maze_file, std::ifstream::in);

  if (fs.good()) {
    AbstractMaze maze(fs);
    ConsoleMouse::inst()->seedMaze(&maze);
    maze.print_maze();
    fs.close();
    return EXIT_SUCCESS;
  } else {
    printf("error opening maze file!\n");
    fs.close();
    return EXIT_FAILURE;
  }
}

