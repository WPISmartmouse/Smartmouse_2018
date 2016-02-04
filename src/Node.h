#pragma once

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
    bool known;
    bool visited;
    int row;
    int col;
    int weight; //used for flood-fill
    int distance; //used for a star

    //if you want to iterate over neighbors, just increment the pointer to north
    Node *neighbors[4];

    static const int OUT_OF_BOUNDS = -2;

    /** \brief intializes a node */
    Node();

    /** \brief get the neighbor in the given direction */
    Node *neighbor(const Direction dir);
};
