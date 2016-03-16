#include "ConsoleMouse.h"

ConsoleMouse::ConsoleMouse(AbstractMaze *maze) : Mouse(maze) {}

ConsoleMouse::ConsoleMouse(AbstractMaze *maze, int starting_row, int starting_col) : Mouse(maze, starting_row, starting_col) {}

SensorReading ConsoleMouse::sense(){
  SensorReading sr(row, col);
  std::array<bool ,4> *w = &sr.walls;
	Node *n = get_mouse_node();

	for (int i=0;i<w->size();i++){
		(*w)[i] = (n->neighbors[i] == 0x0);
	}

  return sr;
}
