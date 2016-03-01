#ifndef A_SOLVER
#define A_SOLVER

#include "WallFollow.h"
#include <string.h>
#include <stdio.h>
#ifndef EMBED
  #include <iostream>
#endif

void WallFollow::setup(KnownMaze *kmaze) {
  this->kmaze = kmaze;
  kmaze->reset();
  kmaze->mark_origin_known();
	step=0;
}

char *WallFollow::solve(){
	//run till you find the goal
	while (!isFinished()){
    stepOnce();
	}
  teardown();
  return kmaze->fastest_route;
}

bool WallFollow::isFinished(){
  return Mouse::atCenter();
}

void WallFollow::teardown() {
	kmaze->fastest_route[step]=0;
}

AbstractMaze WallFollow::stepOnce(){
  Direction dir = left_of_dir(Mouse::getDir());

  if (!kmaze->is_mouse_blocked(dir)){
    printf("turning left\n");
    //if you can turn left you must
    Mouse::turn_to_face(dir);
  }
  else if (kmaze->is_mouse_blocked(Mouse::getDir())) {
    if (!kmaze->is_mouse_blocked(opposite_direction(dir))){
      //if you can't go left or forward try right
      Mouse::turn_to_face(opposite_direction(dir));
    printf("turning right\n");
    }
    else {
      //you must do a 180
      Mouse::turn_to_face(opposite_direction(Mouse::getDir()));
    printf("turning 180\n");
    }
  }

  Mouse::forward();
  kmaze->fastest_route[step++] = dir_to_char(Mouse::getDir());
  kmaze->mark_mouse_position_visited();

  return *kmaze;
}
#endif
