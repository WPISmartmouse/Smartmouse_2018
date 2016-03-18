#include "Flood.h"
#include <string.h>

#ifndef EMBED
#include <iostream>
#else
#include <Arduino.h>
#endif

Flood::Flood(Mouse *mouse) : Solver(mouse),
                                done(false),
                                solvable(true),
                                solved(false) {
	no_wall_path = (char*)calloc(AbstractMaze::PATH_SIZE, sizeof(char));
	all_wall_path = (char *)calloc(AbstractMaze::PATH_SIZE, sizeof(char));
}

//starts at 0, 0 and explores the whole maze
void Flood::setup(){
  mouse->reset();
  mouse->maze->reset();
  no_wall_maze.connect_all_neighbors_in_maze();
  goal = no_wall_maze.center_node();
}

Direction Flood::planNextStep(){
  //mark the nodes visted in both the mazes
  no_wall_maze.mark_position_visited(mouse->getRow(), mouse->getCol());
  all_wall_maze.mark_position_visited(mouse->getRow(), mouse->getCol());

  //check left right back and front sides
  //eventually this will return values from sensors
  SensorReading sr = mouse->checkWalls();

  //update the mazes base on that reading
  no_wall_maze.update(sr);
  all_wall_maze.update(sr);

  //solve flood fill on the two mazes from mouse to goal
  solvable = no_wall_maze.flood_fill_from_point(no_wall_path,
      mouse->getRow(),
      mouse->getCol(),
      goal->row(),
      goal->col());
  all_wall_maze.flood_fill_from_point(all_wall_path,
      mouse->getRow(),
      mouse->getCol(),
      goal->row(),
      goal->col());

  //solve from origin to center
  //this is what tells us whether or not we need to keep searching
  no_wall_maze.flood_fill_from_origin_to_center(no_wall_maze.fastest_route);
  all_wall_maze.flood_fill_from_origin_to_center(all_wall_maze.fastest_route);

  return char_to_dir(no_wall_path[0]);
}

char *Flood::solve(){
	//mouse starts at 0, 0
	while (!isFinished()){
    mouse->internalTurnToFace(planNextStep());
    mouse->internalForward();
  }

  return all_wall_maze.fastest_route;
}

bool Flood::isFinished(){
  return mouse->atCenter();
}

void Flood::teardown(){
	//this is the final solution which represents how the mouse should travel from start to finish
  mouse->maze->fastest_route = all_wall_maze.fastest_route;
	free(no_wall_path);
	free(all_wall_path);
}
