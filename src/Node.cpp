#include "Node.h"

#include <stdlib.h>

Node *Node::neighbor(const Direction dir){
  switch(dir){
    case Direction::N:
      return neighbors[0];
    case Direction::E:
      return neighbors[1];
    case Direction::S:
      return neighbors[2];
    case Direction::W:
      return neighbors[3];
    default: return NULL;
  }
}

Node::Node() : known(false), weight(-1), neighbors{NULL, NULL, NULL, NULL} {
}

