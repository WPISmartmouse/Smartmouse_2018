#include "Mouse.h"
#include "AbstractMaze.h"
#include "Direction.h"

Mouse::Mouse() : row(0), col(0), dir(Direction::S){}

int Mouse::getRow(){
  return row;
}

int Mouse::getCol(){
  return col;
}

Direction Mouse::getDir(){
  return dir;
}

bool Mouse::atCenter(){
	return (row == AbstractMaze::CENTER || row == AbstractMaze::CENTER-1) && (col == AbstractMaze::CENTER || col == AbstractMaze::CENTER-1);
}

bool Mouse::inBounds(){
  return row >= 0 && col >= 0
    && row < AbstractMaze::MAZE_SIZE
    && col < AbstractMaze::MAZE_SIZE;
}

void Mouse::internalTurnToFace(Direction dir) {
  this->dir = dir;
}

void Mouse::internalForward(){
  switch(dir){
    case Direction::N: row--; break;
    case Direction::E: col++; break;
    case Direction::S: row++; break;
    case Direction::W: col--; break;
  }
}

void Mouse::turnToFace(char c){
  turnToFace(char_to_dir(c));
}
