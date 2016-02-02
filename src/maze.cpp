/** maze.c implements the functions used to create, free, and manipulate maze
  @author Peter Mitrano
  */
#include "maze.h"

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

Direction operator++(Direction& dir, int) {
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

Maze::Maze() : solved(false) {
  fastest_route = (char *)malloc(PATH_SIZE*sizeof(char)); //assume really bad route--visits all squares.
  int i,j;
  for (i=0;i<MAZE_SIZE;i++){
    for (j=0;j<MAZE_SIZE;j++){
      nodes[i][j] = new Node();
      nodes[i][j]->row = i;
      nodes[i][j]->col = j;
    }
  }
}

#ifndef EMBED
Maze::Maze(std::fstream& fs){
  fastest_route = (char *)malloc(PATH_SIZE*sizeof(char)); //assume really bad route--visits all squares.
  for (int i=0;i<MAZE_SIZE;i++){
    for (int j=0;j<MAZE_SIZE;j++){
      nodes[i][j] = new Node();
      nodes[i][j]->row = i;
      nodes[i][j]->col = j;
    }
  }

  std::string line;

  //look West and North to connect any nodes
  for (int i=0;i<MAZE_SIZE;i++){ //read in each line
    std::getline(fs, line);

    if (!fs) {
      printf("getline failed\n.");
      return;
    }

    int charPos = 0;
    for (int j=0;j<MAZE_SIZE;j++){
      if (line.at(charPos) != '|'){
        connect_neighbor(i, j, Direction::W);
      }
      charPos++;
      if (line.at(charPos) != '_'){
        connect_neighbor(i, j, Direction::S);
      }
      charPos++;
    }
  }
  printf("\n");
}
#endif

int Maze::get_node(Node **out, int row, int col){
  if (col < 0 || col >= MAZE_SIZE || row < 0 || row >= MAZE_SIZE){
    return Node::OUT_OF_BOUNDS;
  }

  if(nodes[row][col] == NULL){
    return -1;
  }
  (*out) = nodes[row][col];
  return 0;
}

void Maze::update_nodes(int row, int col, bool *walls){
  for (Direction d=Direction::First;d<Direction::Last;d++){
    //
    //if a wall exists in that direction, add a wall
    if (walls[static_cast<int>(d)]){
      connect_neighbor(row, col, d);
    }
    //if no wall exists in that direction remove it
    else {
      //getting the previously unconnected neighbor represents adding a wall
      //make sure to update both the node and the node it now connects to
      remove_neighbor(row, col, d);
    }
  }
}

Node *Maze::maze_diff(Maze *maze2){
  Node *new_goal = NULL;

  int i,j,max = -1;
  for (i=0;i<MAZE_SIZE;i++){
    for (j=0;j<MAZE_SIZE;j++){

      Node *n1;
      this->get_node(&n1, i,j);
      Node *n2;
      maze2->get_node(&n2, i,j);

      //don't look at nodes you've already actually visited
      if (!n1->visited && !n2->visited){
        int count1=0,count2=0;

        Direction d;
        for (d=Direction::First;d<Direction::Last;d++){
          if (n1->neighbor(d) ==  NULL) count1++;
          if (n2->neighbor(d) == NULL) count2++;
        }

        int diff = abs(count2-count1);

        if (diff > max){
          max = diff;
          //doesn't matter n1 or n2, all we're using are the row/col
          new_goal = n1;
        }
      }
    }
  }

  return new_goal;
}

bool Maze::visited(int row, int col, Direction dir){
  return nodes[row][col]->neighbor(dir)->known;
}

int Maze::get_node_in_direction(Node **out, int row, int col, const Direction dir){
  switch(dir){
    case Direction::N:
      return get_node(out, row-1,col);
      break;
    case Direction::E:
      return get_node(out, row,col+1);
      break;
    case Direction::S:
      return get_node(out, row+1,col);
      break;
    case Direction::W:
      return get_node(out, row,col-1);
      break;
    default:
      return -1;
  }
}

void Maze::remove_neighbor(int row, int col, const Direction dir){
  Node *n1;
  get_node(&n1, row, col);
  Node *n2;
  get_node_in_direction(&n2, row, col, dir);
  n1->neighbors[static_cast<int>(dir)] = NULL;
  n2->neighbors[static_cast<int>(dir)] = NULL;
}

void Maze::connect_neighbor(int row, int col, const Direction dir){
  Node *n1;
  int n1_status = get_node(&n1, row, col);
  Node *n2;
  int n2_status = get_node_in_direction(&n2, row, col, dir);

  Direction opposite = opposite_direction(dir);

  if (n1_status != Node::OUT_OF_BOUNDS) {
    n1->neighbors[static_cast<int>(dir)] = n2;
  }
  if (n2_status != Node::OUT_OF_BOUNDS) {
    n2->neighbors[static_cast<int>(opposite)] = n1;
  }
}

void Maze::connect_all_neighbors(int row, int col){
  for (Direction d = Direction::First; d<Direction::Last; d++){
    connect_neighbor(row, col, d);
  }
}

void Maze::connect_all_neighbors_in_maze() {
  int i, j;
  for (i=0;i<MAZE_SIZE;i++){
    for (j=0;j<MAZE_SIZE;j++){
      connect_all_neighbors(i, j);
    }
  }
}

void Maze::mark_known(int row, int col){
  nodes[row][col]->known = true;
}
