#ifdef CONSOLE
#include "ConsoleMouse.h"
#include "AbstractMaze.h"
#include <stdio.h>
#include <stdlib.h>

int ConsoleMouse::forward(){
  internalForward();
	if (row >= AbstractMaze::MAZE_SIZE || row < 0 || col >= AbstractMaze::MAZE_SIZE || col < 0){
    //this is probably the most serious error possible
    //it means you've run into a wall. Just give up.
    printf("RAN INTO A FUCKING WALL\n");
    exit(-1);
	}
  return 0;
}

void ConsoleMouse::turnToFace(Direction d){
  if (d == Direction::INVALID){
    //again, this is a super serious error... you can't ever do this.
    printf("YOU CAN'T TURN THAT WAY\n");
    exit(-1);
  }
	if (dir != d){
    internalTurnToFace(d);
	}
	//in reality this will turn the physical mouse
}
#endif

