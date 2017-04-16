#pragma once

#include <array>
#include "Direction.h"

/**
 * \brief holds its location & neighbors, as well as a bool for indicating if it has been discovered
 * you don't need to free nodes in a maze, just use free_maze however, be sure to free nodes allocated not in mazes
 * I finally gave in an added row and col attributes the node....be very careful you dont get fuck with those.
 * Don't ever move a node around in maze because you risk getting the actual row/col off from the nodes row/col
 * visited is meant for ACTUALLY visiting, known is just used for searching/solving
 */
class Node {
public:
  int weight; //used for flood-fill
  int distance; //used for a star
  bool known;
  bool visited;

  //if you want to iterate over neighbors, just increment the pointer to north
  std::array<Node *, 4> neighbors;

  static const int OUT_OF_BOUNDS;

  /** \brief intializes a node */
  Node(unsigned int row, unsigned int col);

  Node();

  unsigned int row();

  unsigned int col();

  /** \brief get the neighbor in the given direction */
  Node *neighbor(const Direction dir);

  bool wall(const Direction dir);

  void assign_weights_to_neighbors(Node *goal, int weight, bool *success);

private:
  unsigned int r;
  unsigned int c;
};
