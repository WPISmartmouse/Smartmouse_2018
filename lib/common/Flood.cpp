#ifndef A_SOLVER
#define A_SOLVER

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
	final_solution= (char *)calloc(AbstractMaze::PATH_SIZE, sizeof(char));
}

//starts at 0, 0 and explores the whole maze
//kmaze is the known maze,  and should only be used to call sense()
void Flood::setup(){
  mouse->maze->reset();
  no_wall_maze.connect_all_neighbors_in_maze();
}

Direction Flood::planNextStep(){
  //mark the nodes visted in both the mazes
  no_wall_maze.mark_position_visited(mouse->getRow(), mouse->getCol());
  all_wall_maze.mark_position_visited(mouse->getRow(), mouse->getCol());

  //check left right back and front sides
  //eventually this will return values from sensors
  SensorReading sr = mouse->sense();

  //update the mazes base on that reading
  no_wall_maze.update(sr);
  all_wall_maze.update(sr);

  //first priority is look for the goal, so follow the no_wall_path until the goal is found
  //when you're at the goal, the path will be empty.
  if (mouse->getRow() == 7 && mouse->getCol() == 7){
    solved = true;
  }

  //once it has been solved,  change behavior to gather more information about the maze
  //take the all_wall and no_wall mazes and compare them
  //for each node,  check how many walls are the same
  //visit the nodes that have the MOST differences
  Node *goal;
  if (solved){
    //diffs all squares and returns sorted list of nodes to visit
    //the row/col of this goal will be used by flood fill custom
    goal = no_wall_maze.maze_diff(&all_wall_maze);
  }
  else {
    //it doesn't matter which maze this is from
    //all that happens is it's row & col get sent to flood fill custom
    goal = no_wall_maze.center_node();
  }

  //if there aren't any unvisted nodes left,  stop!
  if (goal == NULL){
    done = true;
    no_wall_maze.flood_fill_from_origin(final_solution,
        AbstractMaze::CENTER,
        AbstractMaze::CENTER);
    return Direction::INVALID;
  }
  else {
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
    no_wall_maze.flood_fill_from_origin(no_wall_maze.fastest_route,goal->row(),
        goal->col());
    all_wall_maze.flood_fill_from_origin(all_wall_maze.fastest_route,goal->row(),goal->col());

    return char_to_dir(no_wall_path[0]);
  }
}

char *Flood::solve(){
	//mouse starts at 0, 0
	while (!isFinished()){
    mouse->internalTurnToFace(planNextStep());
    mouse->internalForward();
  }

  return final_solution;
}

bool Flood::isFinished(){
  return strcmp(no_wall_maze.fastest_route, all_wall_maze.fastest_route) == 0 && solvable && done;
}

void Flood::teardown(){
	//this is the final solution which represents how the mouse should travel from start to finish
	free(no_wall_path);
	free(all_wall_path);
  free(final_solution);
}
#endif
