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

Pose ConsoleMouse::getPose() {
  Pose p;
  p.x = row * AbstractMaze::UNIT_DIST + AbstractMaze::HALF_UNIT_DIST;
  p.y = col * AbstractMaze::UNIT_DIST + AbstractMaze::HALF_UNIT_DIST;
  p.yaw = dir_to_yaw(dir);
  return p;
}

double ConsoleMouse::getRowOffsetToEdge() {
  return AbstractMaze::HALF_UNIT_DIST;
}

double ConsoleMouse::getColOffsetToEdge() {
  return AbstractMaze::HALF_UNIT_DIST;
}
