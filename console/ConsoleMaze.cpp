#ifdef CONSOLE

#include "ConsoleMaze.h"

ConsoleMaze::ConsoleMaze(std::fstream &fs) : AbstractMaze() {
  std::string line;

  //look West and North to connect any nodes
  for (int i = 0; i < MAZE_SIZE; i++) { //read in each line
    std::getline(fs, line);

    if (!fs) {
      printf("getline failed\n.");
      return;
    }

    int charPos = 0;
    for (int j = 0; j < MAZE_SIZE; j++) {
      if (line.at(charPos) != '|') {
        connect_neighbor(i, j, Direction::W);
      }
      charPos++;
      if (line.at(charPos) != '_') {
        connect_neighbor(i, j, Direction::S);
      }
      charPos++;
    }
  }
  printf("\n");
}

#endif
