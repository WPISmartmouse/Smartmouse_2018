#include "Flood.h"
#include <string.h>
#ifndef EMBED
  #include <iostream>
#endif

#define min(a,  b) ((a) < (b) ? (a) : (b))

Flood::Flood() : solved(false), solvable(true), done(false) {
	no_wall_path = (char*)calloc(AbstractMaze::PATH_SIZE, sizeof(char));
	all_wall_path = (char *)calloc(AbstractMaze::PATH_SIZE, sizeof(char));
}

//starts at 0, 0 and explores the whole maze
//kmaze is the known maze,  and should only be used to call sense()
void Flood::setup(KnownMaze kmaze){
	//make all connections open in the no_wall_maze
  this->kmaze = kmaze;
  kmaze.reset();
  no_wall_maze.connect_all_neighbors_in_maze();
}

AbstractMaze Flood::stepOnce(){
  int r = Mouse::getRow();
  int c = Mouse::getCol();

  //check left right back and front sides
  //eventually this will return values from sensors
  SensorReading sr = kmaze.sense();

  //update the mazes base on that reading
  no_wall_maze.update(sr);
  all_wall_maze.update(sr);

  //first priority is look for the goal, so follow the no_wall_path until the goal is found
  //when you're at the goal, the path will be empty.
  if (Mouse::getRow() == 7 && Mouse::getCol() == 7){
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

    //if there aren't any unvisted nodes left,  stop!
    if (goal == NULL){
      //exit out of the do loop
      done = true;
    }
  }
  else {
    //it doesn't matter which maze this is from
    //all that happens is it's row & col get sent to flood fill custom
    goal = no_wall_maze.center_node();
  }

  //solve flood fill on the two mazes from mouse to goal
  solvable = no_wall_maze.flood_fill_from_mouse(no_wall_path,goal->row(),goal->col());
  all_wall_maze.flood_fill_from_mouse(all_wall_path,goal->row(),goal->col());
  printf("no wall path: %s\n",no_wall_path);

  //solve from origin to center
  //this is what tells us whether or not we need to keep searching
  no_wall_maze.flood_fill_from_origin(no_wall_maze.fastest_route,goal->row(),
      goal->col());
  all_wall_maze.flood_fill_from_origin(all_wall_maze.fastest_route,goal->row(),goal->col());

  //follow the no_wall path towards the previously defined goal
  //this will be the center if we haven't already found it,
  //or some other node if we have found it
  Mouse::turn_to_face(no_wall_path[0]);
  Mouse::forward();
  no_wall_maze.print_maze_mouse();

  //mark the nodes visted in both the mazes
  no_wall_maze.mark_mouse_position_visited();
  all_wall_maze.mark_mouse_position_visited();
}

char *Flood::solve(){
	//mouse starts at 0, 0
	while (!isFinished()){
    stepOnce();
  }

  return all_wall_maze.fastest_route;
}

bool Flood::isFinished(){
  return strcmp(no_wall_maze.fastest_route, all_wall_maze.fastest_route) == 0 && solvable && done;
}

void Flood::teardown(){
	//this is the final solution which represents how the mouse should travel from start to finish
	free(no_wall_path);
	free(all_wall_path);
}
