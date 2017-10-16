#pragma once

#include <common/core/AbstractMaze.h>
#include <sim/simulator/msgs/maze.pb.h>
#include <sim/simulator/msgs/robot_description.pb.h>
#include <ignition/math.hh>

namespace smartmouse {
namespace msgs {
smartmouse::msgs::Maze Convert(AbstractMaze *maze, std::string name = "", int size = smartmouse::maze::SIZE);

AbstractMaze Convert(smartmouse::msgs::Maze maze_msg);

::Direction Convert(smartmouse::msgs::Direction dir_msg);

::Direction Convert(smartmouse::msgs::Direction::Dir dir_enum);

RobotDescription Convert(std::ifstream &fs);

double ConvertSec(ignition::msgs::Time time);

unsigned long ConvertMSec(ignition::msgs::Time time);

std::vector<ignition::math::Line2d> MazeToLines(smartmouse::msgs::Maze maze);

std::tuple<double, double, double, double> WallToCoordinates(smartmouse::msgs::Wall wall);

}
}
