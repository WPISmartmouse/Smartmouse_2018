#include <cstring>
#include "util.h"
#include "Mouse.h"

double Mouse::meterToRad(double x) {
  return x / WHEEL_RAD;
}

double Mouse::radToMeters(double x) {
  return x * WHEEL_RAD;
}

Mouse::Mouse() : maze(new AbstractMaze()), row(0), col(0), dir(Direction::E) {}

Mouse::Mouse(unsigned int starting_row, unsigned int starting_col) : maze(new AbstractMaze()), row(starting_row), col(starting_col),
                                                   dir(Direction::E) {}

Mouse::Mouse(AbstractMaze *maze) : maze(maze), row(0), col(0), dir(Direction::E) {}

Mouse::Mouse(AbstractMaze *maze, unsigned int starting_row, unsigned int starting_col) : maze(maze), row(starting_row), col(starting_col),
                                                                       dir(Direction::E) {}

void Mouse::reset() {
  row = 0;
  col = 0;
  dir = Direction::E;
}

unsigned int Mouse::getRow() {
  return row;
}

unsigned int Mouse::getCol() {
  return col;
}

Direction Mouse::getDir() {
  return dir;
}

bool Mouse::atCenter() {
  return row == AbstractMaze::CENTER && col == AbstractMaze::CENTER;
}

bool Mouse::inBounds() {
  return row >= 0 && col >= 0
         && row < AbstractMaze::MAZE_SIZE
         && col < AbstractMaze::MAZE_SIZE;
}

void Mouse::internalTurnToFace(Direction dir) {
  this->dir = dir;
}

void Mouse::internalForward() {
  switch (dir) {
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
      break;
  }
}

bool Mouse::isWallInDirection(Direction d) {
  Node *mouse_node;
  maze->get_node(&mouse_node, row, col);
  bool is_wall = mouse_node->neighbor(d) == nullptr;
  return is_wall;
}


void Mouse::mark_mouse_position_visited() {
  maze->nodes[row][col]->visited = true;
}

void Mouse::print_maze_mouse() {
  char buff[AbstractMaze::BUFF_SIZE];
  maze_mouse_string(buff);
  print(buff);
}

void Mouse::maze_mouse_string(char *buff) {
  char *b = buff;
  unsigned int i, j;
  for (i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      Node *n = maze->nodes[i][j];
      if (n->neighbor(Direction::W) == NULL) {
        strcpy(b++, "|");
      } else {
        strcpy(b++, "_");
      }

      if (row == i && col == j) {
        strcpy(b++, "o");
      } else if (n->neighbor(Direction::S) == NULL) {
        strcpy(b++, "_");
      } else {
        strcpy(b++, " ");
      }
    }
    *(b++) = '|';
    *(b++) = '\n';
  }
  b++;
  *b = '\0';
}

