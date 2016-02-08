/** maze.c implements the functions used to create, free, and manipulate maze
  @author Peter Mitrano
  */
#include "AbstractMaze.h"
#include "Mouse.h"
#include <string.h>

AbstractMaze::AbstractMaze() : solved(false) {
  fastest_route = (char *)malloc(PATH_SIZE*sizeof(char)); //assume really bad route--visits all squares.
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
      connect_neighbor(sr.row, sr.col, d);
    }
    //if no wall exists in that direction remove it
    else {
      //getting the previously unconnected neighbor represents adding a wall
      //make sure to update both the node and the node it now connects to
      remove_neighbor(sr.row, sr.col, d);
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
  return nodes[AbstractMaze::MAZE_SIZE/2][AbstractMaze::MAZE_SIZE/2];
}

void AbstractMaze::mark_mouse_position_visited() {
  nodes[Mouse::getRow()][Mouse::getCol()]->visited= true;
}

Node *AbstractMaze::get_mouse_node(){
  return nodes[Mouse::getRow()][Mouse::getCol()];
}

bool AbstractMaze::flood_fill_from_mouse(char *path, int r1,  int c1){
  flood_fill(path, Mouse::getRow(), Mouse::getCol(), r1, c1);
}

bool AbstractMaze::flood_fill_from_origin(char *path, int r1,  int c1){
  flood_fill(path, 0, 0, r1, c1);
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

  //start at the goal
  n = goal;

  //recursively visits all neighbors
  n->assign_weights_to_neighbors(nodes[r0][c0], 0, &success);

  //if we solved the maze,  traverse from goal back to root and record what direction is shortest
  char *r_path = (char *)malloc(AbstractMaze::PATH_SIZE*sizeof(char));
  char *r_p = r_path;
  while (n != nodes[r0][c0]){
    Node *min_node = n;
    Direction min_dir = Direction::N;

    //find the neighbor with the lowest weight and go there,  that is the fastest route
    Direction d;
    for (d=Direction::First;d<Direction::Last;d++){
      if (n->neighbor(d) != NULL){
        if (n->neighbor(d)->weight < min_node->weight){
          min_node = n->neighbor(d);
          min_dir = d;
        }
      }
    }

    n = min_node;

    *(r_p++) = dir_to_char(min_dir);
  }
  *r_p = '\0';
  r_p = r_path;

  //the create path is from goal to start,  so now we "reverse" it
  char* p = path + strlen(r_p);
  *(p--) = '\0';
  while ((*r_p) != '\0'){
    char c;
    switch(*r_p){
      case 'N':c='S';break;
      case 'E':c='W';break;
      case 'S':c='N';break;
      case 'W':c='E';break;
    }
    *(p--) = c;
    r_p++;
  }

  free(r_path);
  return true;
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
    n2->neighbors[static_cast<int>(dir)] = NULL;
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

void AbstractMaze::mark_origin_known(){
  nodes[0][0]->known = true;
}

bool AbstractMaze::is_mouse_blocked(){
  return is_mouse_blocked(Mouse::getDir());
}

bool AbstractMaze::is_mouse_blocked(Direction dir){
  return nodes[Mouse::getRow()][Mouse::getCol()]->neighbor(dir) == NULL;
}

void AbstractMaze::print_maze_mouse(){
  int i,j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    char *str = (char *)malloc((AbstractMaze::MAZE_SIZE * 2 + 1) * sizeof(char));

    char *s=str;
    for (j=0;j<AbstractMaze::MAZE_SIZE;j++){
      Node *n = nodes[i][j];
      if (n->neighbor(Direction::W) == NULL){
        strcpy(s++,"|");
      }
      else {
        strcpy(s++,"_");
      }

      if (Mouse::getRow()  == i && Mouse::getCol() == j){
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

void AbstractMaze::print_maze(){
  int i,j;
  for (i=0;i<AbstractMaze::MAZE_SIZE;i++){
    char *str = (char *)malloc((AbstractMaze::MAZE_SIZE * 2 + 1) * sizeof(char));

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
      printf("%03d ",w);
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
