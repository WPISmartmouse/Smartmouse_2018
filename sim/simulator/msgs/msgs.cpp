#include "msgs.h"

namespace smartmouse {
namespace msgs {

smartmouse::msgs::Maze fromAbstractMaze(AbstractMaze *maze, std::string name, int size) {
  Maze maze_msg;
  maze_msg.set_name(name);
  maze_msg.set_size(size);

  unsigned int r, c;
  for (r = 0; r < AbstractMaze::MAZE_SIZE; r++) {
    for (c = 0; c < AbstractMaze::MAZE_SIZE; c++) {
      Node *n = maze->nodes[r][c];
      if (n->neighbor(::Direction::E) == nullptr) {
        Wall *wall = maze_msg.add_walls();
        smartmouse::msgs::RowCol *node = wall->mutable_node();
        node->set_row(r);
        node->set_col(c);
        wall->set_direction(Direction_Dir_E);
      }
      if (n->neighbor(::Direction::S) == nullptr) {
        Wall *wall = maze_msg.add_walls();
        smartmouse::msgs::RowCol *node = wall->mutable_node();
        node->set_row(r);
        node->set_col(c);
        wall->set_direction(Direction_Dir_S);
      }
    }
  }

  return maze_msg;
}

AbstractMaze toAbstractMaze(smartmouse::msgs::Maze maze_msg) {
  AbstractMaze maze;

  for (Wall wall : maze_msg.walls()) {
    smartmouse::msgs::RowCol node = wall.node();
    smartmouse::msgs::Direction::Dir dir_msg = wall.direction();
    ::Direction dir = dirMsgEnumToDir(dir_msg);
    maze.connect_all_neighbors_in_maze();
    maze.disconnect_neighbor(node.row(), node.col(), dir);
  }

  return maze;
}

::Direction dirMsgToDir(smartmouse::msgs::Direction dir_msg) {
  return dirMsgEnumToDir(dir_msg.direction());
}

::Direction dirMsgEnumToDir(smartmouse::msgs::Direction::Dir dir_enum) {
  switch (dir_enum) {
    case Direction_Dir_N: return ::Direction::N;
    case Direction_Dir_S: return ::Direction::S;
    case Direction_Dir_E: return ::Direction::E;
    case Direction_Dir_W: return ::Direction::W;
  }
  return ::Direction::INVALID;
}

}
}
