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

Node::Node(int row, int col) : r(row), c(col), known(false), weight(-1), neighbors{NULL, NULL, NULL, NULL} {
}

int Node::row(){
  return r;
}

int Node::col(){
  return c;
}


void Node::explore_neighbors(Node *goal,  int weight,  bool *success){
  //check all nodes that are unvisited,  or would be given a lower weight
  if (!this->known || weight < this->weight){
    //don't visiti it again unless you find a shorter path
    this->known = true;

    //check if path to goal node was found
    if (this == goal){
      *success = true;
    }

    //update weight
    this->weight = weight;

    //recursive call to explore each neighbors
    int i;
    for (i=0;i<4;i++){
      if (this->neighbors[i] != NULL){
        this->neighbors[i]->explore_neighbors(goal, weight+1, success);
      }
    }
  }
}
