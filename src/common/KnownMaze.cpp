#include "KnownMaze.h"
#include "Mouse.h"

#ifndef EMBED
KnownMaze::KnownMaze(std::fstream& fs){
  fastest_route = (char *)malloc(PATH_SIZE*sizeof(char)); //assume really bad route--visits all squares.
  for (int i=0;i<MAZE_SIZE;i++){
    for (int j=0;j<MAZE_SIZE;j++){
      nodes[i][j] = new Node(i,j);
    }
  }

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

SensorReading KnownMaze::sense(){
	bool *walls;
  bool *w = walls;
	Node *n = get_mouse_node();

	for (int i=0;i<4;i++){
		*(w++) = (n->neighbors[i] == NULL);
	}

  return SensorReading(Mouse::getRow(),
      Mouse::getCol(),
      walls);
}
#else
SensorReading KnownMaze::sense(){
  //#TODO actual sensor code here
}
#endif

KnownMaze::KnownMaze(){}
