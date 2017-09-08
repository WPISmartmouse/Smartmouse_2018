#include <sim/simulator/lib/json.hpp>
#include "msgs.h"

namespace smartmouse {
namespace msgs {

smartmouse::msgs::Maze Convert(AbstractMaze *maze, std::string name, int size) {
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

  unsigned int i;
  for (i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    Wall *left_col_wall = maze_msg.add_walls();
    smartmouse::msgs::RowCol *left_col_node = left_col_wall->mutable_node();
    left_col_node->set_row(i);
    left_col_node->set_col(0);
    left_col_wall->set_direction(Direction_Dir_W);

    Wall *top_row_wall = maze_msg.add_walls();
    smartmouse::msgs::RowCol *top_row_node = top_row_wall->mutable_node();
    top_row_node->set_row(0);
    top_row_node->set_col(i);
    top_row_wall->set_direction(Direction_Dir_N);
  }

  return maze_msg;
}

AbstractMaze Convert(smartmouse::msgs::Maze maze_msg) {
  AbstractMaze maze;

  maze.connect_all_neighbors_in_maze();
  for (Wall wall : maze_msg.walls()) {
    smartmouse::msgs::RowCol node = wall.node();
    smartmouse::msgs::Direction::Dir dir_msg = wall.direction();
    ::Direction dir = Convert(dir_msg);
    maze.disconnect_neighbor(node.row(), node.col(), dir);
  }

  return maze;
}

::Direction Convert(smartmouse::msgs::Direction dir_msg) {
  return Convert(dir_msg.direction());
}

::Direction Convert(smartmouse::msgs::Direction::Dir dir_enum) {
  switch (dir_enum) {
    case Direction_Dir_N: return ::Direction::N;
    case Direction_Dir_S: return ::Direction::S;
    case Direction_Dir_E: return ::Direction::E;
    case Direction_Dir_W: return ::Direction::W;
  }
  return ::Direction::INVALID;
}

RobotDescription Convert(std::ifstream &fs) {
  nlohmann::json json;
  json << fs;

  RobotDescription robot_description;
  auto footprint = robot_description.mutable_footprint();
  for (auto json_pt : json["footprint"]) {
    auto pt = footprint->Add();
    pt->set_x(json_pt["x"]);
    pt->set_y(json_pt["y"]);
  }

  return robot_description;
}

}
}
