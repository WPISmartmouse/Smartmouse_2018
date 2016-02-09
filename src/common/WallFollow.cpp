#include "WallFollow.h"
#include <string.h>
#ifndef EMBED
  #include <iostream>
#endif

void WallFollow::setup(KnownMaze kmaze) {
  this->kmaze = kmaze;
  kmaze.reset();
  kmaze.mark_origin_known();
}

char *WallFollow::solve(){
	//run till you find the goal
	int step=0;
	while (!isFinished()){
    stepOnce();
	}
  teardown();
  return kmaze.fastest_route;
}

bool WallFollow::isFinished(){
  return Mouse::atCenter();
}

void WallFollow::teardown() {
	kmaze.fastest_route[step]=0;
}

AbstractMaze WallFollow::stepOnce(){
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

  return kmaze;
}
