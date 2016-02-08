#include "AbstractMaze.h"
#include "Node.h"
#include "gtest/gtest.h"

TEST(NodeTest, OutOfBoundsGetNode) {
  AbstractMaze maze;
  Node *n;
  int status = maze.get_node(&n, -1, -1);
  EXPECT_EQ(status, Node::OUT_OF_BOUNDS);

  status = maze.get_node(&n, AbstractMaze::MAZE_SIZE, AbstractMaze::MAZE_SIZE);
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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
