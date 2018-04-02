#include <sim/simulator/lib/common/json.hpp>
#include "msgs.h"

namespace smartmouse {
namespace msgs {

smartmouse::msgs::Maze Convert(AbstractMaze *maze, std::string name, int size) {
  Maze maze_msg;
  maze_msg.set_name(name);
  maze_msg.set_size(size);

  unsigned int r, c;
  for (r = 0; r < smartmouse::maze::SIZE; r++) {
    for (c = 0; c < smartmouse::maze::SIZE; c++) {
      Node *n = maze->nodes[r][c];
      if (n->neighbor(::Direction::E) == nullptr) {
        Wall *wall = maze_msg.add_walls();
        wall->set_row(r);
        wall->set_col(c);
        wall->set_direction(Direction_Dir_E);
      }
      if (n->neighbor(::Direction::S) == nullptr) {
        Wall *wall = maze_msg.add_walls();
        wall->set_row(r);
        wall->set_col(c);
        wall->set_direction(Direction_Dir_S);
      }
    }
  }

  unsigned int i;
  for (i = 0; i < smartmouse::maze::SIZE; i++) {
    Wall *left_col_wall = maze_msg.add_walls();
    left_col_wall->set_row(i);
    left_col_wall->set_col(0);
    left_col_wall->set_direction(Direction_Dir_W);

    Wall *bottom_row_wall = maze_msg.add_walls();
    bottom_row_wall->set_row(0);
    bottom_row_wall->set_col(i);
    bottom_row_wall->set_direction(Direction_Dir_N);
  }

  return maze_msg;
}

AbstractMaze Convert(smartmouse::msgs::Maze maze_msg) {
  AbstractMaze maze;

  maze.connect_all_neighbors_in_maze();
  for (Wall wall : maze_msg.walls()) {
    smartmouse::msgs::Direction::Dir dir_msg = wall.direction();
    ::Direction dir = Convert(dir_msg);
    maze.disconnect_neighbor(wall.row(), wall.col(), dir);
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

  auto motor = robot_description.mutable_motor();
  motor->set_u_static(json["motor"]["u_static"]);
  motor->set_u_kinetic(json["motor"]["u_kinetic"]);
  motor->set_j(json["motor"]["J"]);
  motor->set_b(json["motor"]["b"]);
  motor->set_k(json["motor"]["K"]);
  motor->set_r(json["motor"]["R"]);
  motor->set_l(json["motor"]["L"]);

  auto left_wheel = robot_description.mutable_left_wheel();
  auto left_wheel_pose = left_wheel->mutable_pose();
  left_wheel_pose->set_x(json["left_wheel"]["pose"]["x"]);
  left_wheel_pose->set_y(json["left_wheel"]["pose"]["y"]);
  left_wheel->set_radius(json["left_wheel"]["radius"]);
  left_wheel->set_thickness(json["left_wheel"]["thickness"]);

  auto right_wheel = robot_description.mutable_right_wheel();
  auto right_wheel_pose = right_wheel->mutable_pose();
  right_wheel_pose->set_x(json["right_wheel"]["pose"]["x"]);
  right_wheel_pose->set_y(json["right_wheel"]["pose"]["y"]);
  right_wheel->set_radius(json["right_wheel"]["radius"]);
  right_wheel->set_thickness(json["right_wheel"]["thickness"]);

  auto front = robot_description.mutable_sensors()->mutable_front();
  auto front_p = front->mutable_p();
  front_p->set_x(json["range_sensors"]["front"]["x"]);
  front_p->set_y(json["range_sensors"]["front"]["y"]);
  front_p->set_theta(json["range_sensors"]["front"]["theta"]);
  front->set_a(json["range_sensors"]["front"]["a"]);
  front->set_b(json["range_sensors"]["front"]["b"]);
  front->set_c(json["range_sensors"]["front"]["c"]);

  auto back_right = robot_description.mutable_sensors()->mutable_back_right();
  auto back_right_p = back_right->mutable_p();
  back_right_p->set_x(json["range_sensors"]["back_right"]["x"]);
  back_right_p->set_y(json["range_sensors"]["back_right"]["y"]);
  back_right_p->set_theta(json["range_sensors"]["back_right"]["theta"]);
  back_right->set_a(json["range_sensors"]["back_right"]["a"]);
  back_right->set_b(json["range_sensors"]["back_right"]["b"]);
  back_right->set_c(json["range_sensors"]["back_right"]["c"]);

  auto back_left = robot_description.mutable_sensors()->mutable_back_left();
  auto back_left_p = back_left->mutable_p();
  back_left_p->set_x(json["range_sensors"]["back_left"]["x"]);
  back_left_p->set_y(json["range_sensors"]["back_left"]["y"]);
  back_left_p->set_theta(json["range_sensors"]["back_left"]["theta"]);
  back_left->set_a(json["range_sensors"]["back_left"]["a"]);
  back_left->set_b(json["range_sensors"]["back_left"]["b"]);
  back_left->set_c(json["range_sensors"]["back_left"]["c"]);


  auto gerald_right = robot_description.mutable_sensors()->mutable_gerald_right();
  auto gerald_right_p = gerald_right->mutable_p();
  gerald_right_p->set_x(json["range_sensors"]["gerald_right"]["x"]);
  gerald_right_p->set_y(json["range_sensors"]["gerald_right"]["y"]);
  gerald_right_p->set_theta(json["range_sensors"]["gerald_right"]["theta"]);
  gerald_right->set_a(json["range_sensors"]["gerald_right"]["a"]);
  gerald_right->set_b(json["range_sensors"]["gerald_right"]["b"]);
  gerald_right->set_c(json["range_sensors"]["gerald_right"]["c"]);


  auto gerald_left = robot_description.mutable_sensors()->mutable_gerald_left();
  auto gerald_left_p = gerald_left->mutable_p();
  gerald_left_p->set_x(json["range_sensors"]["gerald_left"]["x"]);
  gerald_left_p->set_y(json["range_sensors"]["gerald_left"]["y"]);
  gerald_left_p->set_theta(json["range_sensors"]["gerald_left"]["theta"]);
  gerald_left->set_a(json["range_sensors"]["gerald_left"]["a"]);
  gerald_left->set_b(json["range_sensors"]["gerald_left"]["b"]);
  gerald_left->set_c(json["range_sensors"]["gerald_left"]["c"]);


  auto front_right = robot_description.mutable_sensors()->mutable_front_right();
  auto front_right_p = front_right->mutable_p();
  front_right_p->set_x(json["range_sensors"]["front_right"]["x"]);
  front_right_p->set_y(json["range_sensors"]["front_right"]["y"]);
  front_right_p->set_theta(json["range_sensors"]["front_right"]["theta"]);
  front_right->set_a(json["range_sensors"]["front_right"]["a"]);
  front_right->set_b(json["range_sensors"]["front_right"]["b"]);
  front_right->set_c(json["range_sensors"]["front_right"]["c"]);


  auto front_left = robot_description.mutable_sensors()->mutable_front_left();
  auto front_left_p = front_left->mutable_p();
  front_left_p->set_x(json["range_sensors"]["front_left"]["x"]);
  front_left_p->set_y(json["range_sensors"]["front_left"]["y"]);
  front_left_p->set_theta(json["range_sensors"]["front_left"]["theta"]);
  front_left->set_a(json["range_sensors"]["front_left"]["a"]);
  front_left->set_b(json["range_sensors"]["front_left"]["b"]);
  front_left->set_c(json["range_sensors"]["front_left"]["c"]);


  return robot_description;
}

double ConvertSec(ignition::msgs::Time time) {
  return time.sec() + (double)(time.nsec()) / 1e9;
}

unsigned long ConvertMSec(ignition::msgs::Time time) {
  return time.sec() * 1000ul + time.nsec() / 1000000ul;
}

void Convert(smartmouse::msgs::Maze maze, maze_walls_t &maze_lines) {
  for (unsigned int r = 0; r < smartmouse::maze::SIZE; r++) {
    for (unsigned int c = 0; c < smartmouse::maze::SIZE; c++) {
      std::vector<smartmouse::msgs::WallPoints> &walls = maze_lines[r][c];
      walls.clear();
    }
  }

  for (auto wall : maze.walls()) {
    double c1, r1, c2, r2;
    std::tie(c1, r1, c2, r2) = WallToCoordinates(wall);
    std::vector<smartmouse::msgs::WallPoints> &walls = maze_lines[wall.row()][wall.col()];
    smartmouse::msgs::WallPoints wall_pts;
    wall_pts.set_c1(c1);
    wall_pts.set_r1(r1);
    wall_pts.set_c2(c2);
    wall_pts.set_r2(r2);
    walls.push_back(wall_pts);
  }
}

std::tuple<double, double, double, double> WallToCoordinates(smartmouse::msgs::Wall wall) {
  double r = wall.row();
  double c = wall.col();

  double c1 = 0, r1 = 0, c2 = 0, r2 = 0;
  switch (wall.direction()) {
    case smartmouse::msgs::Direction_Dir_N: {
      c1 = c - smartmouse::maze::HALF_WALL_THICKNESS_CU;
      r1 = r - smartmouse::maze::HALF_WALL_THICKNESS_CU;
      c2 = c + 1 + smartmouse::maze::HALF_WALL_THICKNESS_CU;
      r2 = r + smartmouse::maze::HALF_WALL_THICKNESS_CU;
      break;
    }
    case smartmouse::msgs::Direction_Dir_S: {
      c1 = c - smartmouse::maze::HALF_WALL_THICKNESS_CU;
      r1 = r + 1 - smartmouse::maze::HALF_WALL_THICKNESS_CU;
      c2 = c + 1 + smartmouse::maze::HALF_WALL_THICKNESS_CU;
      r2 = r + 1 + smartmouse::maze::HALF_WALL_THICKNESS_CU;
      break;
    }
    case smartmouse::msgs::Direction_Dir_E: {
      c1 = c + 1 - smartmouse::maze::HALF_WALL_THICKNESS_CU;
      r1 = r - smartmouse::maze::HALF_WALL_THICKNESS_CU;
      c2 = c + 1 + smartmouse::maze::HALF_WALL_THICKNESS_CU;
      r2 = r + 1 + smartmouse::maze::HALF_WALL_THICKNESS_CU;
      break;
    }
    case smartmouse::msgs::Direction_Dir_W: {
      c1 = c - smartmouse::maze::HALF_WALL_THICKNESS_CU;
      r1 = r - smartmouse::maze::HALF_WALL_THICKNESS_CU;
      c2 = c + smartmouse::maze::HALF_WALL_THICKNESS_CU;
      r2 = r + 1 + smartmouse::maze::HALF_WALL_THICKNESS_CU;
      break;
    }
  }

  return std::tuple<double, double, double, double>{c1, r1, c2, r2};
}

ignition::msgs::Time Convert(int time_millis) {
  ignition::msgs::Time t;
  t.set_sec(time_millis / 1000);
  t.set_nsec((time_millis % 1000) * 1000000);

  return t;
}

}
}
