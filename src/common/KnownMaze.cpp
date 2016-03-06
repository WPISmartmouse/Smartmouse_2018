#include "KnownMaze.h"
#include "Mouse.h"

#ifdef CONSOLE
KnownMaze::KnownMaze(std::fstream& fs, Mouse *mouse) : AbstractMaze(mouse) {
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
  SensorReading sr(mouse->getRow(), mouse->getCol());
  bool *w = sr.walls;
	Node *n = get_mouse_node();

	for (int i=0;i<4;i++){
		*(w++) = (n->neighbors[i] == NULL);
	}

  return sr;
}
#endif
#ifdef SIM
void KnownMaze::sense_callback(ConstLaserScanStampedPtr &msg){
  //transform from Mouse frame to Cardinal frame;
  int size = msg->scan().ranges_size();
  switch(mouse->getDir()) {
    case Direction::N:
      walls[0] = msg->scan().ranges(1) < WALL_DIST;
      walls[1] = msg->scan().ranges(0) < WALL_DIST;
      walls[2] = msg->scan().ranges(3) < WALL_DIST;
      walls[3] = msg->scan().ranges(2) < WALL_DIST;
      break;
    case Direction::E:
      walls[0] = msg->scan().ranges(2) < WALL_DIST;
      walls[1] = msg->scan().ranges(1) < WALL_DIST;
      walls[2] = msg->scan().ranges(0) < WALL_DIST;
      walls[3] = msg->scan().ranges(3) < WALL_DIST;
      break;
    case Direction::S:
      walls[0] = msg->scan().ranges(3) < WALL_DIST;
      walls[1] = msg->scan().ranges(2) < WALL_DIST;
      walls[2] = msg->scan().ranges(1) < WALL_DIST;
      walls[3] = msg->scan().ranges(0) < WALL_DIST;
      break;
    case Direction::W:
      walls[0] = msg->scan().ranges(0) < WALL_DIST;
      walls[1] = msg->scan().ranges(3) < WALL_DIST;
      walls[2] = msg->scan().ranges(2) < WALL_DIST;
      walls[3] = msg->scan().ranges(1) < WALL_DIST;
      break;
  }
  sense_cond.notify_all();
}

SensorReading KnownMaze::sense(){
  std::unique_lock<std::mutex> lk(sense_mutex);
  sense_cond.wait(lk);
  SensorReading sr(mouse->getRow(), mouse->getCol());
  bool *w = sr.walls;
  for (int i=0;i<4;i++){
    *(w++) = walls[i];
  }

  return sr;
}
#endif
#ifdef EMBED
#include <Arduino.h>
SensorReading KnownMaze::sense(){
  //#TODO actual sensor code here
}
#endif

bool KnownMaze::is_mouse_blocked(){
  SensorReading sr = sense();
  return sr.isWall(mouse->getDir());
}

bool KnownMaze::is_mouse_blocked(Direction dir){
  SensorReading sr = sense();
  return sr.isWall(dir);
}

KnownMaze::KnownMaze(Mouse *mouse) : AbstractMaze(mouse) {}
