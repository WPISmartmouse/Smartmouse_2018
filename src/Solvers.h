/** functions for exploring and solving mazes */
#pragma once

#include "Maze.h"
#include "Direction.h"
#include "Node.h"
#include "Mouse.h"
#include "MazeIO.h"
#include <stdbool.h>

/*not used */
void A_star(Maze *maze, int x0, int y0, int x1, int y1);

/*not used */
Node *closest_unknown(Maze *maze);

/** Not used
*/
void update_neighbors(Node *node);

/** \brief starts at 0,0 and explores the whole maze
 * kmaze is the known maze, and is used to "read the sensors"
 * The mouse solves the maze after sensing each new square
 * solves assuming ALL walls, and assuming NO walls
 * when the solution for those two mazes are the same, then it knows the fastest route
 * @param kmaze the known maze. in reality this won't exist because the sensors can read from the real world
 */
void flood_explore(Maze kmaze);

/** set starting point 0,0 & S. depth-first search of all nodes in the grid, adding 1 each time.
* once all nodes are discovered, the path is solved by going backwards from the center to the lowest neighbor each time
* @param maze maze to solve
* @param path an allocate char* of at least PATH_SIZE to store the solution in
*/
bool flood_fill(Maze *maze, char *path);

/** this one also allows you to specify a starting position other tha 0,0
 * @param maze the maze the solve
 * @param path an allocate char* of at least PATH_SIZE to store the solution in
 * @param r0 the row to start in
 * @param c0 the column to start in
 * @param r1 the row to end in
 * @param c1 the column to end in
 */
bool flood_fill_custom(Maze *maze, char *path, int r0, int c0, int r1, int c1);

/**this assigns n->weight+1 to each neighbor, and recursively calls it on all valid neighbors
* @param node the node to explore the neighbors of
* @param goal the node you want to find a path to
* @param weight the weight to set the node to
* @param success should be false to start, and will be set to true if path to goal is found
*/
void explore_neighbors(Node *node,Node *goal, int weight, bool *success);

/** set starting point to 0,0. follow left hand wall
* @param maze maze to solve
*/
void left_hand_follow(Maze maze);

/** marks the mouses given position as known
* @param mouse the mouse
* @param maze the maze
*/
void mark_position(Mouse mouse, Maze *maze);

/** checks the mouses direction, and compares it to the mazes walls. For example, if the mouse is facing north, and the node it's on has a null north pointer, then there's a wall
* @param mouse the mouse
* @param maze the maze
*/
bool blocked(Mouse mouse, Maze *maze);

/** checks a direction, and compares it to the mazes walls. For example, if the mouse is facing north, and the node it's on has a null north pointer, then there's a wall
* @param mouse the mouse
* @param maze the maze
*/
bool blocked_in_dir(int row, int col, Direction dir, Maze *maze);

/** chooses a direction. If possible, it picks an unvisited node, otherwise it picks a random one it's been too already
* @param mouse the mouse
*/
void choose_direction(Mouse mouse, Maze *maze);

/** checks if the mouse is int the center 4 squares of the maze
* @param mouse the mouse
*/
bool atCenter(Mouse mouse);

/** checks if the row col is at center
* @param mouse the mouse
*/
bool isCenter(int row, int col);
