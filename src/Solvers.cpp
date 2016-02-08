/** solvers.h
	@author Peter Mitrano
	use args to the make command to print more information
	ARGS=-DDEBUG  prints just the mouse position
	ARGS=-DDEBUG_FULL prints mazes,  paths,  and mouse positions
	ARGS=-DDEBUG_PATH prints just paths

*/
#include "Solvers.h"
#include <string.h>
#ifndef EMBED
  #include <iostream>
#endif

#define min(a,  b) ((a) < (b) ? (a) : (b))

//starts at 0, 0 and explores the whole maze
//kmaze is the known maze,  and should only be used to call sense()
void Solvers::flood_explore(KnownMaze kmaze){
	AbstractMaze no_wall_maze; //this maze is initially no walls,  and walls are filled out every time the mouse moves
	AbstractMaze all_wall_maze; //this maze is initially no walls,  and walls are removed every time the mouse moves

	//make all connections open in the no_wall_maze
  no_wall_maze.connect_all_neighbors_in_maze();

	char *no_wall_path;
	no_wall_path = (char*)calloc(AbstractMaze::PATH_SIZE, sizeof(char));
	char *all_wall_path;
	all_wall_path = (char *)calloc(AbstractMaze::PATH_SIZE, sizeof(char));

	//exit if no solutions are found
	bool solvable = true;

	//change behavior once goal is found
	bool solved = false;

	//mouse starts at 0, 0
	do {
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
				break;
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

		//solve from origin to center
		//this is what tells us whether or not we need to keep searching
		no_wall_maze.flood_fill_from_origin(no_wall_maze.fastest_route,goal->row(),
        goal->col());
		all_wall_maze.flood_fill_from_origin(all_wall_maze.fastest_route,goal->row(),goal->col());

		//follow the no_wall path towards the previously defined goal
		//this will be the center if we haven't already found it,
		//or some other node if we have found it
    //
		//mouse->execute_command(*no_wall_path);

		//mark the nodes visted in both the mazes
		no_wall_maze.mark_mouse_position_visited();
		all_wall_maze.mark_mouse_position_visited();

		all_wall_maze.print_maze_mouse();
#ifndef EMBED
    std::cin.get();
#endif
	}
	while (strcmp(no_wall_maze.fastest_route, all_wall_maze.fastest_route) != 0 && solvable); //don't let it go forever


	//this is the final solution which represents how the mouse should travel from start to finish
	if (solvable){
		printf("SOLUTION = %s\n", all_wall_maze.fastest_route);
	}
	else {
		printf("NO POSSIBLE SOLUTION\n");
	}

	free(no_wall_path);
	free(all_wall_path);
}

void Solvers::left_hand_follow(KnownMaze kmaze){
  kmaze.mark_origin_known();

	//run till you find the goal
	int step=0;
	while (!Mouse::atCenter()){

    kmaze.print_maze_mouse();

		Direction dir = left_of_dir(Mouse::getDir());

    if (!kmaze.is_mouse_blocked(dir)){
      //if you can left you must
      Mouse::turn_to_face(dir);
    }
    else if (kmaze.is_mouse_blocked(Mouse::getDir())) {
      if (!kmaze.is_mouse_blocked(opposite_direction(dir))){
        //if you can't go left or forward try right
        Mouse::turn_to_face(opposite_direction(dir));
      }
      else {
        //you must do a 180
        Mouse::turn_to_face(opposite_direction(Mouse::getDir()));
      }
    }

    Mouse::forward();
		kmaze.mark_mouse_position_visited();

#ifndef EMBED
    std::cin.get();
    system("clear");
#endif

	}
	kmaze.fastest_route[step]=0;
	printf("solved! %s\n", kmaze.fastest_route);
}
