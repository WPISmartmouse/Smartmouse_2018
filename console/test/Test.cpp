#include <common/core/AbstractMaze.h>
#include <common/core/Direction.h>
#include <console/ConsoleMouse.h>
#include <common/core/Mouse.h>
#include <fstream>
#include <common/core/WallFollow.h>
#include <common/core/Flood.h>
#include <common/core/Node.h>
#include "gtest/gtest.h"

const char *FLOOD_SLN = "1E3S2E2S1W3S2E2N1E1N1E2S2E1S";

const char *WALL_FOLLOW_SLN =
"9E1W1S1N1W2S2N1W3S2E1N1E1N1S1W1S1E2S3E1N2W2E3N1W2S1W1N1S1E2N1W1N1S1E1N1S1E1N3E10S5N1W2S3W1S2W2N2W2N2S1E2N2S1E2S1E2N3E1N1S3W2S1E1N3E3N1W1E1N1W1E1N1W1N1S1E3S1E5N1W1S1N2W5S3W2N3W1S4N1W2S1N1W1S3W1S2E1S1E1N1E1S1N1W1S1W1S1E1W2S2N1W3S2E2N1E1N1E2S1E";

TEST(NodeTest, OutOfBoundsGetNode) {
  AbstractMaze maze;
  Node *n;
  int status = maze.get_node(&n, -1, -1);
  EXPECT_EQ(status, Node::OUT_OF_BOUNDS);

  status = maze.get_node(&n, smartmouse::maze::SIZE, smartmouse::maze::SIZE);
  EXPECT_EQ(status, Node::OUT_OF_BOUNDS);
}

TEST(NodeTest, InBoundsGetNode) {
  AbstractMaze maze;
  Node *n;
  int status = maze.get_node(&n, 0, 0);
  EXPECT_EQ(status, 0);
}

TEST(NeighborTest, NeighborTest) {
  AbstractMaze maze;
  Node *n;
  int status = maze.get_node(&n, 0, 0);
  EXPECT_EQ(status, 0);

  Node *nSouth;
  status = maze.get_node(&nSouth, 1, 0);
  EXPECT_EQ(status, 0);

  Node *neighbor = n->neighbor(Direction::S);
  EXPECT_NE(nSouth, neighbor);

  maze.connect_neighbor(0, 0, Direction::S);

  neighbor = n->neighbor(Direction::S);
  EXPECT_EQ(nSouth, neighbor);
}

TEST(NeighborTest, NeighborOfNeighborIsMyself) {
  AbstractMaze maze;
  maze.connect_all_neighbors(0, 0);
  Node *myself;
  Node *neighbor;
  Node *neighborOfNeighbor;
  int status = maze.get_node(&myself, 0, 0);
  EXPECT_EQ(status, 0);

  maze.connect_neighbor(0, 0, Direction::S);

  neighbor = myself->neighbor(Direction::S);

  ASSERT_NE(neighbor, (Node *)NULL);

  neighborOfNeighbor = neighbor->neighbor(Direction::N);

  EXPECT_EQ(myself, neighborOfNeighbor);
}

TEST(ConnectMazeTest, ConnectAllNeighbors) {
  AbstractMaze maze;
  maze.connect_all_neighbors_in_maze();

  for (unsigned int i=0;i<smartmouse::maze::SIZE;i++){
    for (unsigned int j=0;j<smartmouse::maze::SIZE;j++){
      Node *n;
      int status = maze.get_node(&n, i, j);

      ASSERT_EQ(status, 0);
      ASSERT_NE(n, (Node *)NULL);

      if (i == 0) {
        EXPECT_EQ(n->neighbor(Direction::N), (Node *)NULL);

        if (j == 0) {
          EXPECT_EQ(n->neighbor(Direction::W), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::E), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::S), (Node *)NULL);
        }
        else if (j == smartmouse::maze::SIZE - 1) {
          EXPECT_EQ(n->neighbor(Direction::E), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::W), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::S), (Node *)NULL);
        }
      }
      else if (i == smartmouse::maze::SIZE - 1) {
        EXPECT_EQ(n->neighbor(Direction::S), (Node *)NULL);

        if (j == 0) {
          EXPECT_EQ(n->neighbor(Direction::W), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::E), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::N), (Node *)NULL);
        }
        else if (j == smartmouse::maze::SIZE - 1) {
          EXPECT_EQ(n->neighbor(Direction::E), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::W), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::N), (Node *)NULL);
        }
      }
    }
  }
}

TEST(ConnectMazeTest, RemoveNeighbors) {
  AbstractMaze maze;
  maze.connect_all_neighbors_in_maze();

  Node *n;
  int status = maze.get_node(&n, 0, 0);
  ASSERT_EQ(status, 0);
  ASSERT_NE(n, (Node *)NULL);

  Node *nSouth;
  status = maze.get_node_in_direction(&nSouth, 0, 0, Direction::S);
  ASSERT_EQ(status, 0);
  ASSERT_NE(n, (Node *)NULL);

  maze.disconnect_neighbor(0, 0, Direction::S);
  ASSERT_EQ(n->neighbor(Direction::S), (Node *)NULL);
  ASSERT_EQ(nSouth->neighbor(Direction::N), (Node *)NULL);
}

TEST(FloodFillTest, EmptyMaze){
  std::string maze_file = "../../mazes/empty.mz";
  std::ifstream fs;
  fs.open(maze_file, std::ifstream::in);

  ASSERT_TRUE(fs.good());

  AbstractMaze maze(fs);
  Node *origin;
  Node *center;

  int status = maze.get_node(&origin, 0, 0);
  ASSERT_EQ(status, 0);

  status = maze.get_node(&center,
      smartmouse::maze::SIZE/2,
      smartmouse::maze::SIZE/2);
  ASSERT_EQ(status, 0);

  bool success = false;
  origin->assign_weights_to_neighbors(center, 0, &success);


  for (unsigned int i=0;i<smartmouse::maze::SIZE;i++){
    for (unsigned int j=0;j<smartmouse::maze::SIZE;j++){
      Node *n;
      status = maze.get_node(&n, i, j);

      ASSERT_EQ(status, 0);
      ASSERT_NE(n, (Node *)NULL);

      EXPECT_EQ((signed int)(i+j), n->weight);
    }
  }
}

TEST(FloodFillTest, StripedMaze){
  std::string maze_file = "../../mazes/stripes.mz";
  std::ifstream fs;
  fs.open(maze_file, std::ifstream::in);

  ASSERT_TRUE(fs.good());

  AbstractMaze maze(fs);
  Node *origin;
  Node *center;

  int status = maze.get_node(&origin, 0, 0);
  ASSERT_EQ(status, 0);

  status = maze.get_node(&center,
      smartmouse::maze::SIZE/2,
      smartmouse::maze::SIZE/2);
  ASSERT_EQ(status, 0);

  bool success = false;
  origin->assign_weights_to_neighbors(center, 0, &success);


  for (unsigned int i=0;i<smartmouse::maze::SIZE;i++){
    for (unsigned int j=0;j<smartmouse::maze::SIZE;j++){
      Node *n;
      status = maze.get_node(&n, i, j);

      ASSERT_EQ(status, 0);
      ASSERT_NE(n, (Node *)NULL);

      EXPECT_EQ((signed int)(i+j), n->weight);
    }
  }
}

TEST(SolveMazeTest, WallFollowSolve) {
  std::string maze_file = "../../mazes/16x16.mz";
  std::ifstream fs;
  fs.open(maze_file, std::ifstream::in);

  ASSERT_TRUE(fs.good());

  AbstractMaze maze(fs);
  ConsoleMouse::inst()->seedMaze(&maze);
  WallFollow solver(ConsoleMouse::inst());
  solver.setup();
  route_t solution = solver.solve();
  solver.teardown();

  std::string s = route_to_string(solution);
  EXPECT_STREQ(WALL_FOLLOW_SLN, s.c_str());

  fs.close();
}

TEST(SolveMazeTest, FloodSolve) {
  std::string maze_file = "../../mazes/16x16.mz";
  std::ifstream fs;
  fs.open(maze_file, std::ifstream::in);

  ASSERT_TRUE(fs.good());

  AbstractMaze maze(fs);
  ConsoleMouse::inst()->seedMaze(&maze);
  Flood solver(ConsoleMouse::inst());
  solver.setup();
  route_t solution = solver.solve();
  solver.teardown();

  std::string s = route_to_string(solution);
  EXPECT_STREQ(FLOOD_SLN, s.c_str());

  fs.close();
}

TEST(SolveMazeTest, RandSolve) {
  for (int i=0; i < 100; i++) {
    AbstractMaze maze = AbstractMaze::gen_random_legal_maze();
    ConsoleMouse::inst()->seedMaze(&maze);

    Flood solver(ConsoleMouse::inst());
    solver.setup();
    solver.solve();
    solver.teardown();

    ASSERT_TRUE(solver.isSolvable());
  }
}

TEST(DirectionTest, DirectionLogic) {
  EXPECT_TRUE(Direction::W > Direction::S);
  EXPECT_TRUE(Direction::W > Direction::E);
  EXPECT_TRUE(Direction::W > Direction::N);

  EXPECT_TRUE(Direction::S > Direction::E);
  EXPECT_TRUE(Direction::S > Direction::N);
  EXPECT_TRUE(Direction::S < Direction::W);

  EXPECT_TRUE(Direction::E > Direction::N);
  EXPECT_TRUE(Direction::E < Direction::W);
  EXPECT_TRUE(Direction::E < Direction::S);
}

TEST(RouteUtilTest, RouteUtilAddBack) {
  route_t route;
  insert_motion_primitive_back(&route, {1, Direction::N});
  EXPECT_STREQ(route_to_string(route).c_str(), "1N");
  insert_motion_primitive_back(&route, {1, Direction::N});
  EXPECT_STREQ(route_to_string(route).c_str(), "2N");
  insert_motion_primitive_back(&route, {1, Direction::N});
  EXPECT_STREQ(route_to_string(route).c_str(), "3N");
  insert_motion_primitive_back(&route, {2, Direction::E});
  EXPECT_STREQ(route_to_string(route).c_str(), "3N2E");
  insert_motion_primitive_back(&route, {1, Direction::E});
  EXPECT_STREQ(route_to_string(route).c_str(), "3N3E");
  insert_motion_primitive_back(&route, {1, Direction::S});
  EXPECT_STREQ(route_to_string(route).c_str(), "3N3E1S");
  insert_motion_primitive_back(&route, {1, Direction::E});
  EXPECT_STREQ(route_to_string(route).c_str(), "3N3E1S1E");
}

TEST(RouteUtilTest, RouteUtilAddFront) {
  route_t route;
  insert_motion_primitive_front(&route, {1, Direction::N});
  EXPECT_STREQ(route_to_string(route).c_str(), "1N");
  insert_motion_primitive_front(&route, {1, Direction::N});
  EXPECT_STREQ(route_to_string(route).c_str(), "2N");
  insert_motion_primitive_front(&route, {1, Direction::N});
  EXPECT_STREQ(route_to_string(route).c_str(), "3N");
  insert_motion_primitive_front(&route, {2, Direction::E});
  EXPECT_STREQ(route_to_string(route).c_str(), "2E3N");
  insert_motion_primitive_front(&route, {1, Direction::E});
  EXPECT_STREQ(route_to_string(route).c_str(), "3E3N");
  insert_motion_primitive_front(&route, {1, Direction::S});
  EXPECT_STREQ(route_to_string(route).c_str(), "1S3E3N");
  insert_motion_primitive_front(&route, {1, Direction::E});
  EXPECT_STREQ(route_to_string(route).c_str(), "1E1S3E3N");
}

TEST(RouteStringTest, RouteStringText) {
  route_t route = {{1, Direction::N}, {2, Direction::W}, {3, Direction::E}, {1, Direction::S}};
  std::string s = route_to_string(route);
  EXPECT_STREQ(s.c_str(), "1N2W3E1S");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
