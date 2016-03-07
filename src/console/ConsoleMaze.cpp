#ifdef CONSOLE
#include "ConsoleMaze.h"
#include "ConsoleMouse.h"

ConsoleMaze::ConsoleMaze(ConsoleMouse *mouse) : KnownMaze(mouse) {}

ConsoleMaze::ConsoleMaze(std::fstream& fs, ConsoleMouse *mouse) : ConsoleMaze(mouse) {
  std::string line;

  //look West and North to connect any nodes
  for (int i=0;i<MAZE_SIZE;i++){ //read in each line
    std::getline(fs, line);

    if (!fs) {
      printf("getline failed\n.");
      return;
    }

    int charPos = 0;
    for (int j=0;j<MAZE_SIZE;j++){
      if (line.at(charPos) != '|'){
        connect_neighbor(i, j, Direction::W);
      }
      charPos++;
      if (line.at(charPos) != '_'){
        connect_neighbor(i, j, Direction::S);
      }
      charPos++;
    }
  }
  printf("\n");
}

SensorReading ConsoleMaze::sense(){
  SensorReading sr(mouse->getRow(), mouse->getCol());
  bool *w = sr.walls;
	Node *n = get_mouse_node();

	for (int i=0;i<4;i++){
		*(w++) = (n->neighbors[i] == NULL);
	}

  return sr;
}
#endif
