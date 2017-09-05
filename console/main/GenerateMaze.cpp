#include <console/ConsoleMaze.h>
#include <console/ConsoleMouse.h>
#include <iostream>

int main(int argc, char *argv[]) {

  srand(time(0));

  std::string maze_file;
  std::fstream fs;
  bool save = false;
  if (argc == 2) {
    maze_file = argv[1];

    fs.open(maze_file, std::fstream::out);

    if (fs.good()) {
      save = true;
      std::cout << "saving to file: [" << maze_file << "]" << std::endl;
    } else {
      std::cout << "error opening file: [" << maze_file << "]" << std::endl;
      fs.close();
      return EXIT_FAILURE;
    }
  }

  //generate maze
  AbstractMaze maze = AbstractMaze::gen_random_legal_maze();

  maze.print_maze();

  if (save) {
    std::string buff;
    buff.resize(AbstractMaze::BUFF_SIZE);
    maze.print_maze_str(&buff[0]);
    fs << buff << std::endl;
    fs.close();
  }
}

