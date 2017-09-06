#include <fstream>

#include "gtest/gtest.h"
#include <common/AbstractMaze.h>
#include <msgs/direction.pb.h>

TEST(DirectionTest, DirectionConversion) {
  smartmouse::msgs::Direction dir_msg;

  EXPECT_EQ(dir_msg.direction(), smartmouse::msgs::Direction_Dir_N);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
