#include "AbstractMaze.h"
#include "Node.h"
#include "gtest/gtest.h"

class FooTest : public ::testing::Test {
 protected:

  FooTest() {
    // You can do set-up work for each test here.
  }

  virtual ~FooTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for Foo.
};

TEST(NodeTest, OutOfBounds) {
  AbstractMaze maze;
  Node *n;
  int status = maze.get_node(&n, -1, -1);
  EXPECT_EQ(status, Node::OUT_OF_BOUNDS);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
