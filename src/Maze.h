/** maze.h
 * \brief Contains functions for creating and using the Maze and Node structs
 */
#pragma once

#define MAZE_SIZE (16)
#define PATH_SIZE (100)

#ifndef EMBED
  #include <fstream>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "Node.h"
#include "Direction.h"

class Mouse;

/**
 * \brief the maze is graph of nodes, stored internally as an matrix.
 * don't forget to call free_maze(maze) after a maze is done being used
 */
class Maze {
  public:

    bool solved; //boolean for if we know the fastest route
    Node *nodes[MAZE_SIZE][MAZE_SIZE]; // array of node pointers
    char *fastest_route; //a char array like NSEWNENNSNE, which means North, South, East...

    /** \brief allocates and initializes a node
     * allocates a maze of the given size and sets all links in graph to be null. Naturally, it's column major.
     */
    Maze();

#ifndef EMBED
    /** overall function to create a maze from file. any error will result in a null return.  * @return a valid maze pointer to the maze struct, null on any sort of error
    */
    Maze(std::fstream& fs);
#endif

    /** \brief check if a node in a direction is visited
     * looks up (row,col) in maze and checks its neighbors[dir] and returns if that neighbor is known
     */
    bool visited(int row, int col,  Direction dir);

    /** goes through all the squares in two mazes and compares how many neighbors exist.
     * the highest-difference unvisted node
     */
     Node *maze_diff(Maze *maze2);

    /** \brief get node by its position
     * \return 0 on success, OUT_OF_BOUNDS, or -1 on NULL
     */
    int get_node(Node **out, int x, int y);

    /** \brief get neighbor node in a direction from a position
     * \param the adress of the node to set
     * \param row starting row
     * \param col starting col
     * \param dir the direction of the neighbor you want
     * \return 0 on success, OUT_OF_BOUNDS, or -1 on NULL
     */
    int get_node_in_direction(Node **out, int row, int col, const Direction dir);

    /** \brief connect all neighbors in the whole maze
     * \param i row
     * \param j col
     */
    void connect_all_neighbors_in_maze();

    /** \brief add the neighbor in the given direction
     * \param dir direction connect in
     */
    void connect_neighbor(int row, int col, const Direction dir);

    /** \brief add all the neighbors
     */
    void connect_all_neighbors(int row, int col);

    /** \brief remove any neighbor in the given direction
     * \param dir direction connect in
     */
    void remove_neighbor(int row, int col, const Direction dir);

    /** \brief mark the position a known
     * \param row row
     * \param col col
     */
    void mark_known(int row, int col);

    /**
     * \brief uses the information from a sensor read and correctly adds or removes walls from nodes
     * \param row row
     * \param col col
     * \param walls the array of walls
     * \param n the node to update the neighbors of
     */
    void update_nodes(int row, int col, bool *walls);

    bool blocked_in_dir(Mouse mouse, Direction dir);
};
