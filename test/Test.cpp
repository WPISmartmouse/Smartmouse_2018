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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
