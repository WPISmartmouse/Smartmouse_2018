#include "KnownMaze.h"
#include "Mouse.h"

#ifdef CONSOLE
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
#endif

#ifdef CONSOLE
SensorReading KnownMaze::sense(){
  SensorReading sr(Mouse::getRow(), Mouse::getCol());
  bool *w = sr.walls;
	Node *n = get_mouse_node();

	for (int i=0;i<4;i++){
		*(w++) = (n->neighbors[i] == NULL);
	}

  return sr;
}
#endif
#ifdef SIM
SensorReading KnownMaze::sense(){
  //#TODO actual sensor code here
}
#endif
#ifdef EMBED
SensorReading KnownMaze::sense(){
  //#TODO actual sensor code here
}
#endif

bool KnownMaze::is_mouse_blocked(){
  SensorReading sr = sense();
  return sr.isWall(Mouse::getDir());
}

bool KnownMaze::is_mouse_blocked(Direction dir){
  SensorReading sr = sense();
  return sr.isWall(dir);
}

KnownMaze::KnownMaze(){
  fastest_route = (char *)malloc(PATH_SIZE*sizeof(char));
  int i,j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){
      nodes[i][j] = new Node(i,j);
    }
  }
}
