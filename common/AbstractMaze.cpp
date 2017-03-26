/** maze.c implements the functions used to create, free, and manipulate maze
  @author Peter Mitrano
  */
#include "AbstractMaze.h"
#include "Mouse.h"
#include <string.h>
#include <stdlib.h>

#ifdef EMBED
#include <Arduino.h>
#endif

const float AbstractMaze::UNIT_DIST = 0.18;

AbstractMaze::AbstractMaze() : solved(false) {
  fastest_route = (char *)malloc(PATH_SIZE*sizeof(char));
  fastest_theoretical_route = (char *)malloc(PATH_SIZE*sizeof(char));
  pathToNextGoal = (char *)malloc(PATH_SIZE*sizeof(char));
  int i,j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){
      nodes[i][j] = new Node(i,j);
    }
  }
}

int AbstractMaze::get_node(Node **out, int row, int col){
  if (col < 0 || col >= MAZE_SIZE || row < 0 || row >= MAZE_SIZE){
    return Node::OUT_OF_BOUNDS;
  }

  if(nodes[row][col] == NULL){
    return -1;
  }
  (*out) = nodes[row][col];
  return 0;
}

int AbstractMaze::get_node_in_direction(Node **out, int row, int col, const Direction dir){
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

void AbstractMaze::reset() {
  int i, j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){
      nodes[i][j]->weight = -1;
      nodes[i][j]->known = false;
    }
  }
}

void AbstractMaze::update(SensorReading sr){
  for (Direction d=Direction::First;d<Direction::Last;d++){
    //if a wall exists in that direction, add a wall
    if (sr.isWall(d)){
      remove_neighbor(sr.row, sr.col, d);
    }
    //if no wall exists in that direction remove it
    else {
      //getting the previously unconnected neighbor represents adding a wall
      //make sure to update both the node and the node it now connects to
      connect_neighbor(sr.row, sr.col, d);
    }
  }
}

Node *AbstractMaze::maze_diff(AbstractMaze *maze2){
  Node *new_goal = NULL;

  int i,j,max = -1;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){

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

Node *AbstractMaze::center_node(){
  return nodes[AbstractMaze::CENTER][AbstractMaze::CENTER];
}

bool AbstractMaze::flood_fill_from_point(char *path, int r0, int c0, int r1,  int c1){
  return flood_fill(path, r0, c0, r1, c1);
}

bool AbstractMaze::flood_fill_from_origin(char *path, int r1,  int c1){
  return flood_fill(path, 0, 0, r1, c1);
}

bool AbstractMaze::flood_fill_from_origin_to_center(char *path){
  return flood_fill(path, 0, 0, AbstractMaze::MAZE_SIZE/2, AbstractMaze::MAZE_SIZE/2);
}

bool AbstractMaze::flood_fill(char *path, int r0, int c0, int r1,  int c1){
  Node *n;
  Node *goal = nodes[r1][c1];

  //incase the maze has already been solved,  reset all weight and known values
  reset();

  if (r0 == r1 && c0 == c1){
    return true;
  }

  //explore all neighbors of the current node starting  with a weight of 1
  //return 1 means path to goal was found
  bool success = false;
  bool solvable = true;

  //start at the goal
  n = goal;

  //recursively visits all neighbors
  nodes[r0][c0]->assign_weights_to_neighbors(n, 0, &success);

  //if we solved the maze,  traverse from goal back to root and record what direction is shortest
  char *p = path;
  while (n != nodes[r0][c0] && solvable){
    Node *min_node = n;
    Direction min_dir = Direction::N;

    //find the neighbor with the lowest weight and go there,  that is the fastest route
    Direction d;
    bool deadend = true;
    for (d=Direction::First;d<Direction::Last;d++){
      if (n->neighbor(d) != NULL){
        if (n->neighbor(d)->weight < min_node->weight){
          min_node = n->neighbor(d);
          min_dir = d;
          deadend = false;
        }
      }
    }

    if (deadend) {
      solvable = false;
    }

    n = min_node;

    *(p++) = dir_to_char(min_dir);
  }
  *p = '\0';

  if (solvable) {
    //!!! TODO this part is probably wrong

    //the create path is from goal to start,  so now we "reverse" it
    int j = 0;
    int hi = strlen(path) - 1;
    int mid = strlen(path)/2;
    for (int i = hi; i >= mid; i--) {
      char tmp = opposite_direction(path[i]);
      path[i] = opposite_direction(path[j]);
      path[j] = tmp;
      j++;
    }
  }

  return solvable;
}

void AbstractMaze::remove_neighbor(int row, int col, const Direction dir){
  Node *n1;
  int n1_status = get_node(&n1, row, col);
  Node *n2;
  int n2_status = get_node_in_direction(&n2, row, col, dir);

  if (n1_status != Node::OUT_OF_BOUNDS){
    n1->neighbors[static_cast<int>(dir)] = NULL;
  }

  if (n2_status != Node::OUT_OF_BOUNDS){
    n2->neighbors[static_cast<int>(opposite_direction(dir))] = NULL;
  }
}

void AbstractMaze::connect_neighbor(int row, int col, const Direction dir){
  Node *n1;
  int n1_status = get_node(&n1, row, col);
  Node *n2;
  int n2_status = get_node_in_direction(&n2, row, col, dir);

  Direction opposite = opposite_direction(dir);

  if ((n1_status != Node::OUT_OF_BOUNDS) && (n2_status != Node::OUT_OF_BOUNDS)) {
    n1->neighbors[static_cast<int>(dir)] = n2;
    n2->neighbors[static_cast<int>(opposite)] = n1;
  }
}

void AbstractMaze::connect_all_neighbors(int row, int col){
  for (Direction d = Direction::First; d<Direction::Last; d++){
    connect_neighbor(row, col, d);
  }
}

void AbstractMaze::connect_all_neighbors_in_maze() {
  int i, j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){
      connect_all_neighbors(i, j);
    }
  }
}

void AbstractMaze::mark_position_visited(int row, int col){
  nodes[row][col]->visited = true;
}

void AbstractMaze::mark_origin_known(){
  nodes[0][0]->known = true;
}

void AbstractMaze::print_maze(){
  int i,j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    char *str = (char *)malloc((AbstractMaze::MAZE_SIZE * 2 + 2) * sizeof(char));

    char *s=str;
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){
      Node *n = nodes[i][j];
      if (n->neighbor(Direction::W) == NULL){
        strcpy(s++,"|");
        if (n->neighbor(Direction::S) == NULL){
          strcpy(s++,"_");
        }
        else {
          strcpy(s++," ");
        }
      }
      else {
        strcpy(s++,"_");
        if (n->neighbor(Direction::S) == NULL){
          strcpy(s++,"_");
        }
        else {
          strcpy(s++," ");
        }
      }
    }
    *(s++) = '|';
    *s = '\0';
    printf("%s\n",str);
    free(str);
  }
}

void AbstractMaze::print_neighbor_maze(){
  int i,j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){
      for (Direction d=Direction::First;d<Direction::Last;d++){
        bool wall = (nodes[i][j]->neighbor(d) == NULL);
        printf("%i",wall);
      }
      printf(" ");
    }
    printf("\n");
  }
}

void AbstractMaze::print_weight_maze(){
  int i,j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){
      int w = nodes[i][j]->weight;
      printf("%03u ",w);
    }
    printf("\n");
  }
}

void AbstractMaze::print_dist_maze(){
  int i,j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){
      Node *n = nodes[i][j];
      int d = n->distance;
      if (d<10){
        printf("  %d ",d);
      }
      else if (d<100){
        printf(" %d ",d);
      }
      else {
        printf("%d ",d);
      }
    }
    printf("\n");
  }
}

void AbstractMaze::print_pointer_maze(){
  int i,j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){
      printf("%p ", nodes[i][j]);
    }
    printf("\n");
  }
}
