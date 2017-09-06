#pragma once

#include <common/AbstractMaze.h>
#include <sim/simulator/msgs/maze.pb.h>

namespace smartmouse {
namespace msgs {
smartmouse::msgs::Maze fromAbstractMaze(AbstractMaze *maze, std::string name = "", int size = AbstractMaze::MAZE_SIZE);

AbstractMaze toAbstractMaze(smartmouse::msgs::Maze maze_msg);

::Direction dirMsgToDir(smartmouse::msgs::Direction dir_msg);

::Direction dirMsgEnumToDir(smartmouse::msgs::Direction::Dir dir_enum);
}
}
