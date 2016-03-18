#include "Mouse.h"
#include "AbstractMaze.h"
#include "Direction.h"

const float Mouse::ROT_TOLERANCE = 0.01;

Mouse::Mouse() : maze(new AbstractMaze()), row(0), col(0), dir(Direction::S){}

Mouse::Mouse(int starting_row, int starting_col) :
  maze(new AbstractMaze()),
  row(starting_row),
  col(starting_col),
  dir(Direction::S) {}

void Mouse::reset() {
  row = 0;
  col = 0;
  dir = Direction::S;
}

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
	return row == AbstractMaze::CENTER && col == AbstractMaze::CENTER;
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
    default: break;
  }
}

bool Mouse::is_mouse_blocked(){
  SensorReading sr = checkWalls();
  return sr.isWall(dir);
}

bool Mouse::is_mouse_blocked(Direction dir){
  SensorReading sr = checkWalls();
  return sr.isWall(dir);
}

void Mouse::mark_mouse_position_visited() {
  maze->nodes[row][col]->visited= true;
}

Node *Mouse::get_mouse_node(){
  return maze->nodes[row][col];
}

#ifdef EMBED
void Mouse::print_maze_mouse(){}
#else
#include <stdlib.h>
#include <string.h>

void Mouse::print_maze_mouse(){
  int i,j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    char *str = (char *)malloc((AbstractMaze::MAZE_SIZE * 2 + 2) * sizeof(char));

    char *s=str;
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){
      Node *n = maze->nodes[i][j];
      if (n->neighbor(Direction::W) == NULL){
        strcpy(s++,"|");
      }
      else {
        strcpy(s++,"_");
      }

      if (row  == i && col == j){
        strcpy(s++,"o");
      }
      else if (n->neighbor(Direction::S) == NULL){
        strcpy(s++,"_");
      }
      else {
        strcpy(s++," ");
      }
    }
    *(s++) = '|';
    *s = '\0';
    printf("%s\n",str);
    free(str);
  }
}
#endif
