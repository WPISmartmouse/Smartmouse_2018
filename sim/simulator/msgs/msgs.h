#pragma once

#include <common/core/AbstractMaze.h>
#include <sim/simulator/msgs/maze.pb.h>
#include <sim/simulator/msgs/robot_description.pb.h>
#include <ignition/math.hh>

namespace smartmouse {
namespace msgs {
smartmouse::msgs::Maze Convert(AbstractMaze *maze, std::string name = "", int size = smartmouse::maze::SIZE);

AbstractMaze Convert(smartmouse::msgs::Maze maze_msg);


typedef std::vector<smartmouse::msgs::WallPoints> maze_walls_t[smartmouse::maze::SIZE][smartmouse::maze::SIZE];
void Convert(smartmouse::msgs::Maze maze, maze_walls_t &maze_lines);

::Direction Convert(smartmouse::msgs::Direction dir_msg);

::Direction Convert(smartmouse::msgs::Direction::Dir dir_enum);

RobotDescription Convert(std::ifstream &fs);

double ConvertSec(ignition::msgs::Time time);

unsigned long ConvertMSec(ignition::msgs::Time time);

ignition::msgs::Time Convert(int time_millis);

std::tuple<double, double, double, double> WallToCoordinates(smartmouse::msgs::Wall wall);

}
}
