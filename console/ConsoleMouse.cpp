#include "ConsoleMouse.h"

ConsoleMouse *ConsoleMouse::instance = nullptr;

ConsoleMouse *ConsoleMouse::inst() {
  if (instance == NULL) {
    instance = new ConsoleMouse();
  }

  return instance;
}

void ConsoleMouse::seedMaze(AbstractMaze *maze) { this->true_maze = maze; }

ConsoleMouse::ConsoleMouse() {}

ConsoleMouse::ConsoleMouse(int starting_row, int starting_col)
        : Mouse(starting_row, starting_col) {}

SensorReading ConsoleMouse::checkWalls() {
  SensorReading sr(row, col);
  std::array<bool, 4> *w = &sr.walls;
  Node *n = true_maze->nodes[row][col];

  for (unsigned int i = 0; i < w->size(); i++) {
    (*w)[i] = (n->neighbors[i] == 0x0);
  }

  return sr;
}

GlobalPose ConsoleMouse::getGlobalPose() {
  GlobalPose p;
  p.row = 0.5 + row;
  p.col = 0.5 + col;
  p.yaw = dir_to_yaw(dir);
  return p;
}
LocalPose ConsoleMouse::getLocalPose() {
  LocalPose p;
  p.to_left = 0.5;
  p.to_back = 0.5;
  p.yaw_from_straight = dir_to_yaw(dir);
  return p;
}
