/** these functions read in a .mz file and parse it into a valid maze*/
#ifndef READ_MAZE_H
#define READ_MAZE_H

#include "maze.h"
#include "mouse.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


/** overall function to create a maze from file. any error will result in a null return.
* @return a valid maze pointer to the maze struct, null on any sort of error
*/
Maze *read_from_file(FILE *f);

/** prints a maze
* @param maze the maze
*/
void print_maze(Maze *maze);


/** prints a maze with a mouse in it
* @param mouse the mouse
* @param maze the maze
*/
void print_maze_mouse(Maze *maze, Mouse *mouse);


/** duh*/
void print_pointer_maze(Maze *maze);

/** prints each node as a list of booleans
EX)  0010 would mean on wall South
     1011 would mean walls to the North, South, and West

*/
void print_neighbor_maze(Maze *maze);

/** duh*/
void print_weight_maze(Maze *maze);

/** duh*/
void print_dist_maze(Maze *maze);

#endif
