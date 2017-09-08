#pragma once

#include <common/AbstractMaze.h>
#include <sim/simulator/msgs/maze.pb.h>
#include <sim/simulator/msgs/robot_description.pb.h>

namespace smartmouse {
namespace msgs {
smartmouse::msgs::Maze FromAbstractMaze(AbstractMaze *maze, std::string name = "", int size = AbstractMaze::MAZE_SIZE);

AbstractMaze ToAbstractMaze(smartmouse::msgs::Maze maze_msg);

::Direction DirMsgToDir(smartmouse::msgs::Direction dir_msg);

::Direction Convert(smartmouse::msgs::Direction::Dir dir_enum);

RobotDescription Convert(std::ifstream &fs);
}
}
