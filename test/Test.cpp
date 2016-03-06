#include "AbstractMaze.h"
#include "Direction.h"
#include "KnownMaze.h"
#include "ConsoleMouse.h"
#include <fstream>
#include "WallFollow.h"
#include "Flood.h"
#include "Node.h"
#include "gtest/gtest.h"

const char *FLOOD_SLN = "SSSSSSSSSSSSSSSEEEEEEENNNNNNWSSSWWWWWNNNNNNESSEENNENESSEES";

const char *WALL_FOLLOW_SLN = "SSSSSSSSSSSSSSSEEEEEEENNNNNNWSSSSSWNSENNWWWWWSSEEENWWEESWWWNNNNNNNNENWEENWWENWNNSENSEENNWSNWWEEEEEEEEWSNWSSNNWSSSEENENSWSESSEEENWWEENNNWSSWNSENNWNSENSENEEESSSSSSSSSSNNNNNWSSWWWSWWNNWWNNSSENNSSESSENNEEENSWWWSSENEEENNNWENWENWNSESSSENNNNNWSNWWSSSSSWWWNNWWWSNNNNWSSNWSWWWSEESENESNWSWSEWSSNNWSSSEENNENESSE";

TEST(NodeTest, OutOfBoundsGetNode) {
  AbstractMaze maze(NULL);
  Node *n;
  int status = maze.get_node(&n, -1, -1);
  EXPECT_EQ(status, Node::OUT_OF_BOUNDS);

  status = maze.get_node(&n, AbstractMaze::MAZE_SIZE, AbstractMaze::MAZE_SIZE);
  EXPECT_EQ(status, Node::OUT_OF_BOUNDS);
}

TEST(NodeTest, InBoundsGetNode) {
  AbstractMaze maze(NULL);
  Node *n;
  int status = maze.get_node(&n, 0, 0);
  EXPECT_EQ(status, 0);
}

TEST(NeighborTest, NeighborTest) {
  AbstractMaze maze(NULL);
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
  AbstractMaze maze(NULL);
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
  AbstractMaze maze(NULL);
  maze.connect_all_neighbors_in_maze();

  for (int i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (int j=0;j<AbstractMaze::MAZE_SIZE;j++){
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
        else if (j == AbstractMaze::MAZE_SIZE - 1) {
          EXPECT_EQ(n->neighbor(Direction::E), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::W), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::S), (Node *)NULL);
        }
      }
      else if (i == AbstractMaze::MAZE_SIZE - 1) {
        EXPECT_EQ(n->neighbor(Direction::S), (Node *)NULL);

        if (j == 0) {
          EXPECT_EQ(n->neighbor(Direction::W), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::E), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::N), (Node *)NULL);
        }
        else if (j == AbstractMaze::MAZE_SIZE - 1) {
          EXPECT_EQ(n->neighbor(Direction::E), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::W), (Node *)NULL);
          EXPECT_NE(n->neighbor(Direction::N), (Node *)NULL);
        }
      }
    }
  }
}

TEST(ConnectMazeTest, RemoveNeighbors) {
  AbstractMaze maze(NULL);
  maze.connect_all_neighbors_in_maze();

  Node *n;
  int status = maze.get_node(&n, 0, 0);
  ASSERT_EQ(status, 0);
  ASSERT_NE(n, (Node *)NULL);

  Node *nSouth;
  status = maze.get_node_in_direction(&nSouth, 0, 0, Direction::S);
  ASSERT_EQ(status, 0);
  ASSERT_NE(n, (Node *)NULL);

  maze.remove_neighbor(0,0,Direction::S);
  ASSERT_EQ(n->neighbor(Direction::S), (Node *)NULL);
  ASSERT_EQ(nSouth->neighbor(Direction::N), (Node *)NULL);
}

TEST(FloodFillTest, EmptyMaze){
  std::string maze_file = "../mazes/empty.mz";
  std::fstream fs;
  fs.open(maze_file, std::fstream::in);

  ASSERT_TRUE(fs.good());

  ConsoleMouse mouse;
  KnownMaze maze(fs, &mouse);
  Node *origin;
  Node *center;

  int status = maze.get_node(&origin, 0, 0);
  ASSERT_EQ(status, 0);

  status = maze.get_node(&center,
      AbstractMaze::MAZE_SIZE/2,
      AbstractMaze::MAZE_SIZE/2);
  ASSERT_EQ(status, 0);

  bool success = false;
  origin->assign_weights_to_neighbors(center, 0, &success);


  for (int i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (int j=0;j<AbstractMaze::MAZE_SIZE;j++){
      Node *n;
      int status = maze.get_node(&n, i, j);

      ASSERT_EQ(status, 0);
      ASSERT_NE(n, (Node *)NULL);

      EXPECT_EQ(i+j, n->weight);
    }
  }
}

TEST(FloodFillTest, StripedMaze){
  std::string maze_file = "../mazes/stripes.mz";
  std::fstream fs;
  fs.open(maze_file, std::fstream::in);

  ASSERT_TRUE(fs.good());

  ConsoleMouse mouse;
  KnownMaze maze(fs, &mouse);
  Node *origin;
  Node *center;

  int status = maze.get_node(&origin, 0, 0);
  ASSERT_EQ(status, 0);

  status = maze.get_node(&center,
      AbstractMaze::MAZE_SIZE/2,
      AbstractMaze::MAZE_SIZE/2);
  ASSERT_EQ(status, 0);

  bool success = false;
  origin->assign_weights_to_neighbors(center, 0, &success);


  for (int i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (int j=0;j<AbstractMaze::MAZE_SIZE;j++){
      Node *n;
      int status = maze.get_node(&n, i, j);

      ASSERT_EQ(status, 0);
      ASSERT_NE(n, (Node *)NULL);

      EXPECT_EQ(i+j, n->weight);
    }
  }
}

TEST(SolveMazeTest, WallFollowSolve) {
  std::string maze_file = "../mazes/16x16.mz";
  std::fstream fs;
  fs.open(maze_file, std::fstream::in);

  ASSERT_TRUE(fs.good());

  ConsoleMouse mouse;
  KnownMaze maze(fs, &mouse);
  WallFollow solver(&maze);
  solver.setup();
  char *solution = solver.solve();
  solver.teardown();

  ASSERT_NE(solution, (char *)NULL);
  EXPECT_STREQ(WALL_FOLLOW_SLN, solution);

  fs.close();
}

TEST(SolveMazeTest, FloodSolve) {
  std::string maze_file = "../mazes/16x16.mz";
  std::fstream fs;
  fs.open(maze_file, std::fstream::in);

  ASSERT_TRUE(fs.good());

  ConsoleMouse mouse;
  KnownMaze maze(fs, &mouse);
  Flood solver(&maze);
  solver.setup();
  char *solution = solver.solve();
  solver.teardown();

  ASSERT_NE(solution, (char *)NULL);
  EXPECT_STREQ(FLOOD_SLN, solution);

  fs.close();
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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
