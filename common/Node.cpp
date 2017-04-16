#include "Node.h"

const int Node::OUT_OF_BOUNDS = -2;

Node *Node::neighbor(const Direction dir) {
  switch (dir) {
    case Direction::N:
      return neighbors[0];
    case Direction::E:
      return neighbors[1];
    case Direction::S:
      return neighbors[2];
    case Direction::W:
      return neighbors[3];
    default:
      return 0;
  }
}

Node::Node() : weight(32767), distance(0), known(false), visited(false), neighbors({}) {
}

Node::Node(unsigned int row, unsigned int col) : weight(-1), distance(0), known(false), visited(false), neighbors({}),
                                                 r(row),
                                                 c(col) {
}

unsigned int Node::row() {
  return r;
}

unsigned int Node::col() {
  return c;
}


void Node::assign_weights_to_neighbors(Node *goal, int weight, bool *success) {
  //check all nodes that are unvisited, or would be given a lower weight
  if (!this->known || weight < this->weight) {
    //don't visit it again unless you find a shorter path
    this->known = true;

    //check if path to goal node was found
    if (this == goal) {
      *success = true;
    }

    //update weight
    this->weight = weight;

    //recursive call to explore each neighbors
    int i;
    for (i = 0; i < 4; i++) {
      if (this->neighbors[i] != 0) {
        this->neighbors[i]->assign_weights_to_neighbors(goal, weight + 1, success);
      }
    }
  }
}

bool Node::wall(const Direction dir) {
  return neighbor(dir) == nullptr;
}
