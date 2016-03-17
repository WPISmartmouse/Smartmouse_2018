#include "Direction.h"
#include <stdio.h>

#define M_PI 3.14159265358979

char opposite_direction(char c){
  switch(c){
    case 'N':
      return 'S';
    case 'E':
      return 'W';
    case 'S':
      return 'N';
    case 'W':
      return 'E';
    default:
      return '\0';
  }
}

Direction left_of_dir(Direction dir) {
  switch(dir){
    case Direction::N:
      return Direction::W;
    case Direction::E:
      return Direction::N;
    case Direction::S:
      return Direction::E;
    case Direction::W:
      return Direction::S;
    default:
      return Direction::INVALID;
  }
}

Direction right_of_dir(Direction dir) {
  switch(dir){
    case Direction::N:
      return Direction::E;
    case Direction::E:
      return Direction::S;
    case Direction::S:
      return Direction::W;
    case Direction::W:
      return Direction::N;
    default:
      return Direction::INVALID;
  }
}

Direction operator--(Direction& dir, int) {
  switch(dir){
    case Direction::N:
      dir = Direction::W;
      break;
    case Direction::E:
      dir = Direction::N;
      break;
    case Direction::S:
      dir = Direction::E;
      break;
    case Direction::W:
      dir = Direction::S;
      break;
    case Direction::Last:
      dir = Direction::W;
    default:
      dir = Direction::INVALID;
  }
  return dir;
}

Direction operator++(Direction& dir, int) {
  switch(dir){
    case Direction::N:
      dir = Direction::E;
      break;
    case Direction::E:
      dir = Direction::S;
      break;
    case Direction::S:
      dir = Direction::W;
      break;
    case Direction::W:
      dir = Direction::Last;
      break;
    default:
      dir = Direction::INVALID;
  }
  return dir;
}

double toYaw(Direction d){
  switch(d){
    case Direction::N: return 0;
    case Direction::E: return -M_PI/2;
    case Direction::S: return M_PI;
    case Direction::W: return M_PI/2;
    default: return -999;
  }
}

Direction opposite_direction(Direction d){
  switch(d){
    case Direction::N:
      return Direction::S;
    case Direction::E:
      return Direction::W;
    case Direction::S:
      return Direction::N;
    case Direction::W:
      return Direction::E;
    default:
      return Direction::INVALID;
  }
}

char dir_to_char(Direction dir){
  switch(dir){
    case Direction::N: return 'N';
    case Direction::S: return 'S';
    case Direction::E: return 'E';
    case Direction::W: return 'W';
    default: return '\0';
  }
}

Direction char_to_dir(char c){
  switch(c) {
    case 'N': return Direction::N;
    case 'S': return Direction::S;
    case 'E': return Direction::E;
    case 'W': return Direction::W;
    default: return Direction::INVALID;
  }
}

