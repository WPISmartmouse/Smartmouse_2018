#include "Mouse.h"
#include "AbstractMaze.h"

int Mouse::row = 0;
int Mouse::col = 0;
Direction Mouse::dir = Direction::S;

int Mouse::getRow(){
  return Mouse::row;
}

int Mouse::getCol(){
  return Mouse::col;
}

Direction Mouse::getDir(){
  return Mouse::dir;
}

bool Mouse::atCenter(){
	return (row == AbstractMaze::MAZE_SIZE/2 || row == AbstractMaze::MAZE_SIZE/2-1) && (col == AbstractMaze::MAZE_SIZE/2 || col == AbstractMaze::MAZE_SIZE/2-1);
}

#ifndef EMBED
int Mouse::forward(){
  switch(dir){
    case Direction::N:
      row--;
      break;
    case Direction::E:
      col++;
      break;
    case Direction::S:
      row++;
      break;
    case Direction::W:
      col--;
      break;
    default:
      return -2;
  }

	if (row >= AbstractMaze::MAZE_SIZE || row < 0 || col >= AbstractMaze::MAZE_SIZE || col < 0){
    //this is probably the most serious error possible
    //it means you've run into a wall. Just give up.
    exit(-1);
	}
  return 0;
}
#else
int Mouse::forward(){
  //#TODO ACTUAL MOUSE MOVE CODE HERE
  return 0;
}
#endif

void Mouse::turn_to_face(char c){
  turn_to_face(char_to_dir(c));
}
void Mouse::turn_to_face(Direction d){
  if (d == Direction::INVALID){
    //again, this is a super serious error... you can't ever do this.
    exit(-1);
  }
	if (dir != d){
		dir = d;
	}
	//in reality this will turn the physical mouse
}
