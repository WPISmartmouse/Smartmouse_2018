/** these functions read in a .mz file and parse it into a valid maze*/
#pragma once

#include "Maze.h"
#include "Mouse.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/** prints a maze
* @param maze the maze
*/
void print_maze(Maze maze);


/** prints a maze with a mouse in it
* @param mouse the mouse
* @param maze the maze
*/
void print_maze_mouse(Maze maze, Mouse mouse);


/** duh*/
void print_pointer_maze(Maze maze);

/** prints each node as a list of booleans
EX)  0010 would mean on wall South
     1011 would mean walls to the North, South, and West

*/
void print_neighbor_maze(Maze maze);

/** duh*/
void print_weight_maze(Maze maze);

/** duh*/
void print_dist_maze(Maze maze);
