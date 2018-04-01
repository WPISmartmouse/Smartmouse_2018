/** maze.h
 * \brief Contains functions for creating and using the AbstractMaze and Node structs
 */
#pragma once

#include <stdio.h>
#include <sstream>
#include <vector>

#ifndef ARDUINO
#include <fstream>
#endif

#include "SensorReading.h"
#include "Node.h"
#include "Direction.h"

/**
 * \brief the maze is graph of nodes, stored internally as an matrix.
 * don't forget to call free_maze(maze) after a maze is done being used
 */

struct motion_primitive_t {
  uint8_t n;
  Direction d;

  std::string to_string() {
    std::stringstream ss;
    ss << (int) n << dir_to_char(d);
    return ss.str();
  }
};
typedef std::vector<motion_primitive_t> route_t;

std::string route_to_string(route_t &route);

void insert_motion_primitive_front(route_t *route, motion_primitive_t prim);

void insert_motion_primitive_back(route_t *route, motion_primitive_t prim);

namespace smartmouse {
namespace maze {

constexpr static unsigned int SIZE = 16;
const unsigned long BUFF_SIZE = (SIZE * 2 + 3) * SIZE;
constexpr static unsigned int CENTER = SIZE / 2;
constexpr static double UNIT_DIST_M = 0.18;
constexpr static double WALL_THICKNESS_M = 0.012;
constexpr static double HALF_WALL_THICKNESS_M = WALL_THICKNESS_M / 2.0;
constexpr static double HALF_UNIT_DIST = UNIT_DIST_M / 2.0;
constexpr static double SIZE_M = SIZE * UNIT_DIST_M;

constexpr double toMeters(double cu) {
  return cu * UNIT_DIST_M;
}

constexpr double toCellUnits(double meters) {
  return meters / UNIT_DIST_M;
}

constexpr double WALL_THICKNESS_CU = toCellUnits(WALL_THICKNESS_M);
constexpr double HALF_WALL_THICKNESS_CU = toCellUnits(HALF_WALL_THICKNESS_M);
constexpr double SIZE_CU = toCellUnits(SIZE_M);

}
}

class AbstractMaze {
  friend class Mouse;

 public:

  bool solved;
  route_t fastest_route;
  route_t fastest_theoretical_route;
  route_t path_to_next_goal;

  /** \brief allocates and initializes a node
   * allocates a maze of the given size and sets all links in graph to be null. Naturally, it's column major.
   */
  AbstractMaze();

#ifndef ARDUINO // this can't exist on arduino

  AbstractMaze(std::ifstream &fs);

#endif

  void mark_origin_known();

  void mark_position_visited(unsigned int row, unsigned int col);

  /** \brief add the neighbor in the given direction
   * \param dir direction connect in
   */
  void connect_neighbor(unsigned int row, unsigned int col, const Direction dir);

  void reset();

  /**
   * \brief uses the information from a sensor read and correctly adds or removes walls from nodes
   * \param row row
   * \param col col
   * \param walls the array of walls
   * \param n the node to update the neighbors of
   */
  void update(SensorReading sr);

  //This method will take a maze and perform a traditional flood fill
  //the fill starts from r0, c0 and ends at r1, c1
  bool flood_fill_from_point(route_t *path, unsigned int r0, unsigned int c0, unsigned int r1, unsigned int c1);

  bool flood_fill_from_origin(route_t *path, unsigned int r1, unsigned int c1);

  bool flood_fill_from_origin_to_center(route_t *path);

  /** \brief connect all neighbors in the whole maze
   * \param i row
   * \param j col
   */
  void connect_all_neighbors_in_maze();

  /** \brief get node by its position
   * \return 0 on success, OUT_OF_BOUNDS, or -1 on NULL
   */
  int get_node(Node **out, unsigned int r, unsigned int c);

  /** \brief get neighbor node in a direction from a position
   * \param the adress of the node to set
   * \param row starting row
   * \param col starting col
   * \param dir the direction of the neighbor you want
   * \return 0 on success, OUT_OF_BOUNDS, or -1 on NULL
   */
  int get_node_in_direction(Node **out, unsigned int row, unsigned int col, const Direction dir);

  /** \brief add all the neighbors
   */
  void connect_all_neighbors(unsigned int row, unsigned int col);

  /** \brief disconnect any neighbor in the given direction
   * \param dir direction connect in
   */
  void disconnect_neighbor(unsigned int row, unsigned int col, const Direction dir);

  /** \brief walks along a route in the maze and return the longest valid path
   * Valid means you don't walk through any walls.
   */
  route_t truncate(unsigned int row, unsigned int col, Direction dir, route_t route);

  /** prints a maze
  * @param maze the maze
  */
  void print_maze();

  void print_maze_str(char *buff);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

  /** duh*/
  void print_pointer_maze();

  /** prints each node as a list of booleans
  EX)  0010 would mean on wall South
       1011 would mean walls to the North, South, and West

  */
  void print_neighbor_maze();

  /** duh*/
  void print_weight_maze();

  /** duh*/
  void print_dist_maze();

#pragma clang diagnostic pop

  static AbstractMaze gen_random_legal_maze();

  static void _make_connections(AbstractMaze *maze, Node *node);

  bool flood_fill(route_t *path, unsigned int r0, unsigned int c0, unsigned int r1, unsigned int c1);

  bool operator==(const AbstractMaze &other) const;

  Node *nodes[smartmouse::maze::SIZE][smartmouse::maze::SIZE]; // array of node pointers
};
