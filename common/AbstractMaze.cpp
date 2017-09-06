/** maze.c implements the functions used to create, free, and manipulate maze
  @author Peter Mitrano
  */
#include "AbstractMaze.h"

#include <string.h>
#include <string>
#include <algorithm>
#include <random>

#ifdef EMBED
#include <Arduino.h>
#endif

AbstractMaze::AbstractMaze() : solved(false) {
  fastest_route = (char *) calloc(PATH_SIZE, sizeof(char));
  fastest_theoretical_route = (char *) calloc(PATH_SIZE, sizeof(char));
  path_to_next_goal = (char *) calloc(PATH_SIZE, sizeof(char));

  unsigned int i, j;
  for (i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      nodes[i][j] = new Node(i, j);
    }
  }
}

#ifndef ARDUINO
AbstractMaze::AbstractMaze(std::ifstream &fs) : AbstractMaze() {
  std::string line;

  //look West and North to connect any nodes
  for (unsigned int i = 0; i < MAZE_SIZE; i++) { //read in each line
    std::getline(fs, line);

    if (!fs) {
      printf("getline failed\n.");
      return;
    }

    int charPos = 0;
    for (unsigned int j = 0; j < MAZE_SIZE; j++) {
      if (line.at(charPos) != '|') {
        connect_neighbor(i, j, Direction::W);
      }
      charPos++;
      if (line.at(charPos) != '_') {
        connect_neighbor(i, j, Direction::S);
      }
      charPos++;
    }
  }
  printf("\n");
}
#endif

int AbstractMaze::get_node(Node **out, unsigned int row, unsigned int col) {
  if (col < 0 || col >= MAZE_SIZE || row < 0 || row >= MAZE_SIZE) {
    return Node::OUT_OF_BOUNDS;
  }

  if (nodes[row][col] == nullptr) {
    return -1;
  }
  (*out) = nodes[row][col];
  return 0;
}

int AbstractMaze::get_node_in_direction(Node **out, unsigned int row, unsigned int col, const Direction dir) {
  switch (dir) {
    case Direction::N:
      return get_node(out, row - 1, col);
      break;
    case Direction::E:
      return get_node(out, row, col + 1);
      break;
    case Direction::S:
      return get_node(out, row + 1, col);
      break;
    case Direction::W:
      return get_node(out, row, col - 1);
      break;
    default:
      return -1;
  }
}

void AbstractMaze::reset() {
  unsigned int i, j;
  for (i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      nodes[i][j]->weight = -1;
      nodes[i][j]->known = false;
    }
  }
}

void AbstractMaze::update(SensorReading sr) {
  for (Direction d = Direction::First; d < Direction::Last; d++) {
    //if a wall exists in that direction, add a wall
    if (sr.isWall(d)) {
      disconnect_neighbor(sr.row, sr.col, d);
    }
      //if no wall exists in that direction remove it
    else {
      //getting the previously unconnected neighbor represents adding a wall
      //make sure to update both the node and the node it now connects to
      connect_neighbor(sr.row, sr.col, d);
    }
  }
}

bool AbstractMaze::flood_fill_from_point(char *path, unsigned int r0, unsigned int c0, unsigned int r1, unsigned int c1) {
  return flood_fill(path, r0, c0, r1, c1);
}

bool AbstractMaze::flood_fill_from_origin_to_center(char *path) {
  return flood_fill(path, 0, 0, AbstractMaze::MAZE_SIZE / 2, AbstractMaze::MAZE_SIZE / 2);
}

bool AbstractMaze::flood_fill(char *path, unsigned int r0, unsigned int c0, unsigned int r1, unsigned int c1) {
  Node *n;
  Node *goal = nodes[r1][c1];

  //incase the maze has already been solved,  reset all weight and known values
  reset();

  if (r0 == r1 && c0 == c1) {
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
  while (n != nodes[r0][c0] && solvable) {
    Node *min_node = n;
    Direction min_dir = Direction::N;

    //find the neighbor with the lowest weight and go there,  that is the fastest route
    Direction d;
    bool deadend = true;
    for (d = Direction::First; d < Direction::Last; d++) {
      if (n->neighbor(d) != nullptr) {
        if (n->neighbor(d)->weight < min_node->weight) {
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
    int mid = strlen(path) / 2;
    for (int i = hi; i >= mid; i--) {
      char tmp = opposite_direction(path[i]);
      path[i] = opposite_direction(path[j]);
      path[j] = tmp;
      j++;
    }
  }

  return solvable;
}

void AbstractMaze::disconnect_neighbor(unsigned int row, unsigned int col, const Direction dir) {
  Node *n1 = nullptr;
  int n1_status = get_node(&n1, row, col);
  Node *n2 = nullptr;
  int n2_status = get_node_in_direction(&n2, row, col, dir);

  if (n1_status != Node::OUT_OF_BOUNDS) {
    n1->neighbors[static_cast<int>(dir)] = nullptr;
  }

  if (n2_status != Node::OUT_OF_BOUNDS) {
    n2->neighbors[static_cast<int>(opposite_direction(dir))] = nullptr;
  }
}

void AbstractMaze::connect_neighbor(unsigned int row, unsigned int col, const Direction dir) {
  Node *n1 = nullptr;
  int n1_status = get_node(&n1, row, col);
  Node *n2 = nullptr;
  int n2_status = get_node_in_direction(&n2, row, col, dir);

  Direction opposite = opposite_direction(dir);

  if ((n1_status != Node::OUT_OF_BOUNDS) && (n2_status != Node::OUT_OF_BOUNDS)) {
    n1->neighbors[static_cast<int>(dir)] = n2;
    n2->neighbors[static_cast<int>(opposite)] = n1;
  }
}

void AbstractMaze::connect_all_neighbors(unsigned int row, unsigned int col) {
  for (Direction d = Direction::First; d < Direction::Last; d++) {
    connect_neighbor(row, col, d);
  }
}

void AbstractMaze::connect_all_neighbors_in_maze() {
  unsigned int i, j;
  for (i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      connect_all_neighbors(i, j);
    }
  }
}

void AbstractMaze::mark_position_visited(unsigned int row, unsigned int col) {
  nodes[row][col]->visited = true;
}

void AbstractMaze::mark_origin_known() {
  nodes[0][0]->known = true;
}

void AbstractMaze::print_maze_str(char *buff) {
  char *b = buff;
  unsigned int i, j;
  for (i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      Node *n = nodes[i][j];
      if (n->neighbor(Direction::W) == nullptr) {
        strcpy(b++, "|");
        if (n->neighbor(Direction::S) == nullptr) {
          strcpy(b++, "_");
        } else {
          strcpy(b++, " ");
        }
      } else {
        strcpy(b++, "_");
        if (n->neighbor(Direction::S) == nullptr) {
          strcpy(b++, "_");
        } else {
          strcpy(b++, " ");
        }
      }
    }
    *(b++) = '|';
    *(b++) = '\n';
  }
  b++;
  *b = '\0';
}

void AbstractMaze::print_maze() {
  char buff[BUFF_SIZE];
  print_maze_str(buff);
  print(buff);
}

void AbstractMaze::print_neighbor_maze() {
  unsigned int i, j;
  for (i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      for (Direction d = Direction::First; d < Direction::Last; d++) {
        bool wall = (nodes[i][j]->neighbor(d) == nullptr);
        print("%i", wall);
      }
      print(" ");
    }
    print("\n");
  }
}

void AbstractMaze::print_weight_maze() {
  unsigned int i, j;
  for (i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      int w = nodes[i][j]->weight;
      print("%03u ", w);
    }
    print("\n");
  }
}

void AbstractMaze::print_dist_maze() {
  unsigned int i, j;
  for (i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      Node *n = nodes[i][j];
      int d = n->distance;
      if (d < 10) {
        print("  %d ", d);
      } else if (d < 100) {
        print(" %d ", d);
      } else {
        print("%d ", d);
      }
    }
    print("\n");
  }
}

void AbstractMaze::print_pointer_maze() {
  unsigned int i, j;
  for (i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      print("%p ", nodes[i][j]);
    }
    print("\n");
  }
}

AbstractMaze AbstractMaze::gen_random_legal_maze() {
  AbstractMaze maze;

  // start at center and move out, marking visited nodes as we go
  maze.mark_position_visited(MAZE_SIZE / 2, MAZE_SIZE / 2);
  maze.mark_position_visited(MAZE_SIZE / 2 - 1, MAZE_SIZE / 2);
  maze.mark_position_visited(MAZE_SIZE / 2, MAZE_SIZE / 2 - 1);
  maze.mark_position_visited(MAZE_SIZE / 2 - 1, MAZE_SIZE / 2 - 1);

  // pick std::random start node of the four possible ones;
  unsigned int starting_row = MAZE_SIZE / 2 - std::rand() % 2;
  unsigned int starting_col = MAZE_SIZE / 2 - std::rand() % 2;
  Node *start_node = nullptr;
  maze.get_node(&start_node, starting_row, starting_col);
  _make_connections(&maze, start_node);

  // knock down some more randomly
  unsigned int i = 0;
  while (i < AbstractMaze::MAZE_SIZE * AbstractMaze::MAZE_SIZE / 5) {
    unsigned int row = std::rand() % (AbstractMaze::MAZE_SIZE - 2) + 1;
    unsigned int col = std::rand() % (AbstractMaze::MAZE_SIZE - 2) + 1;
    int d = std::rand() % 4;
    Direction dir = int_to_dir(d);

    // check if that's a valid wall to knock down
    bool can_delete = false;

    switch (dir) {
      case Direction::N: {
        Node *left = nullptr;
        Node *right = nullptr;
        Node *above = nullptr;
        maze.get_node(&left, row, col - 1);
        maze.get_node(&right, row, col + 1);
        maze.get_node(&above, row - 1, col);
        if ((left->wall(Direction::N) || left->wall(Direction::E) || above->wall(Direction::W)) &&
            (right->wall(Direction::N) || right->wall(Direction::W) || above->wall(Direction::E))) {
          can_delete = true;
        }
      }
        break;
      case Direction::E: {
        Node *below = nullptr;
        Node *right = nullptr;
        Node *above = nullptr;
        maze.get_node(&below, row + 1, col);
        maze.get_node(&right, row, col + 1);
        maze.get_node(&above, row - 1, col);
        if ((above->wall(Direction::S) || above->wall(Direction::E) || right->wall(Direction::N)) &&
            (below->wall(Direction::N) || below->wall(Direction::E) || right->wall(Direction::S))) {
          can_delete = true;
        }
      }
        break;
      case Direction::S: {
        Node *left = nullptr;
        Node *right = nullptr;
        Node *below = nullptr;
        maze.get_node(&left, row, col - 1);
        maze.get_node(&right, row, col + 1);
        maze.get_node(&below, row + 1, col);
        if ((left->wall(Direction::S) || left->wall(Direction::E) || below->wall(Direction::W)) &&
            (right->wall(Direction::S) || right->wall(Direction::W) || below->wall(Direction::E))) {
          can_delete = true;
        }
      }
        break;
      case Direction::W: {
        Node *below = nullptr;
        Node*left = nullptr;
        Node *above = nullptr;
        maze.get_node(&below, row + 1, col);
        maze.get_node(&left, row, col - 1);
        maze.get_node(&above, row - 1, col);
        if ((above->wall(Direction::S) || above->wall(Direction::W) || left->wall(Direction::N)) &&
            (below->wall(Direction::N) || below->wall(Direction::W) || left->wall(Direction::S))) {
          can_delete = true;
        }
      }
        break;
      default:
          break;
    }

    if (can_delete) {
      maze.connect_neighbor(row, col, dir);
      i++;
    }
  }

  // knock down center square
  maze.connect_neighbor(MAZE_SIZE / 2, MAZE_SIZE / 2, Direction::N);
  maze.connect_neighbor(MAZE_SIZE / 2, MAZE_SIZE / 2, Direction::W);
  maze.connect_neighbor(MAZE_SIZE / 2 - 1, MAZE_SIZE / 2 - 1, Direction::S);
  maze.connect_neighbor(MAZE_SIZE / 2 - 1, MAZE_SIZE / 2 - 1, Direction::E);

  return maze;
}

void AbstractMaze::_make_connections(AbstractMaze *maze, Node *node) {
  static std::random_device rd;
  static std::mt19937 g(rd());

  unsigned int r = node->row();
  unsigned int c = node->col();
  maze->mark_position_visited(r, c);

  // shuffle directions
  std::vector<Direction> dirs = {Direction::N, Direction::E, Direction::S, Direction::W};
  std::shuffle(dirs.begin(), dirs.end(), g);

  for (auto d : dirs) {
    Node *neighbor;
    int result = maze->get_node_in_direction(&neighbor, r, c, d);
    if (result != Node::OUT_OF_BOUNDS && result != -1) {
      if (!neighbor->visited) {
        maze->connect_neighbor(r, c, d);
        _make_connections(maze, neighbor);
      }
    }
  }
}

bool AbstractMaze::operator==(const AbstractMaze &other) const {
  unsigned int r, c;
  for (r = 0; r < AbstractMaze::MAZE_SIZE; r++) {
    for (c = 0; c < AbstractMaze::MAZE_SIZE; c++) {
      Node *n = nodes[r][c];
      Node *n2 = other.nodes[r][c];
      for (Direction d = Direction::First; d < Direction::Last; d++) {
        bool wall = (n->neighbor(d) == nullptr);
        bool wall2 = (n2->neighbor(d) == nullptr);
        if (wall ^ wall2) {
          return false;
        }
      }
    }
  }
  return true;
}
