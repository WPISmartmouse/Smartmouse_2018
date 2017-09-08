#pragma once

#include <common/AbstractMaze.h>
#include <sim/simulator/msgs/maze.pb.h>
#include <sim/simulator/msgs/robot_description.pb.h>

namespace smartmouse {
namespace msgs {
smartmouse::msgs::Maze Convert(AbstractMaze *maze, std::string name = "", int size = AbstractMaze::MAZE_SIZE);

AbstractMaze Convert(smartmouse::msgs::Maze maze_msg);

::Direction Convert(smartmouse::msgs::Direction dir_msg);

::Direction Convert(smartmouse::msgs::Direction::Dir dir_enum);

RobotDescription Convert(std::ifstream &fs);
}
}
