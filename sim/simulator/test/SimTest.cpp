#include <fstream>

#include "gtest/gtest.h"
#include <common/AbstractMaze.h>
#include <msgs/direction.pb.h>
#include <msgs/msgs.h>
#include <ignition/transport.hh>
#include <sim/simulator/msgs/server_control.pb.h>
#include <lib/common/TopicNames.h>
#include <sim/simulator/lib/Server.h>
#include <msgs/world_statistics.pb.h>
#include <lib/common/RayTracing.h>

TEST(DirectionTest, DirectionMsgConversion) {
  smartmouse::msgs::Direction dir_msg;
  EXPECT_EQ(dir_msg.direction(), smartmouse::msgs::Direction_Dir_N);

  dir_msg.set_direction(smartmouse::msgs::Direction_Dir_N);
  EXPECT_EQ(dir_msg.direction(), smartmouse::msgs::Direction_Dir_N);

  dir_msg.set_direction(smartmouse::msgs::Direction_Dir_E);
  EXPECT_EQ(dir_msg.direction(), smartmouse::msgs::Direction_Dir_E);

  dir_msg.set_direction(smartmouse::msgs::Direction_Dir_S);
  EXPECT_EQ(dir_msg.direction(), smartmouse::msgs::Direction_Dir_S);

  dir_msg.set_direction(smartmouse::msgs::Direction_Dir_W);
  EXPECT_EQ(dir_msg.direction(), smartmouse::msgs::Direction_Dir_W);
}

TEST(DirectionTest, DirectionEnumConversion) {
  EXPECT_EQ(smartmouse::msgs::Convert(smartmouse::msgs::Direction_Dir_N), ::Direction::N);
  EXPECT_EQ(smartmouse::msgs::Convert(smartmouse::msgs::Direction_Dir_E), ::Direction::E);
  EXPECT_EQ(smartmouse::msgs::Convert(smartmouse::msgs::Direction_Dir_S), ::Direction::S);
  EXPECT_EQ(smartmouse::msgs::Convert(smartmouse::msgs::Direction_Dir_W), ::Direction::W);
}

TEST(MazeTest, MazeConversion) {
  AbstractMaze maze;
  smartmouse::msgs::Maze maze_msg = smartmouse::msgs::Convert(&maze);

  EXPECT_EQ(maze_msg.walls_size(),
            (int) (2 * (AbstractMaze::MAZE_SIZE * AbstractMaze::MAZE_SIZE + AbstractMaze::MAZE_SIZE)));

  AbstractMaze maze2 = smartmouse::msgs::Convert(maze_msg);

  EXPECT_EQ(maze, maze2);
}

TEST(ServerTest, QuitTest) {
  ignition::transport::Node node;
  auto server_pub = node.Advertise<smartmouse::msgs::ServerControl>(TopicNames::kServerControl);
  Server server;
  server.Start();
  while (!server.IsConnected());
  smartmouse::msgs::ServerControl quit_msg;
  quit_msg.set_quit(true);
  server_pub.Publish(quit_msg);
  server.Join();
  ASSERT_TRUE(true);
}

TEST(ServerTest, StepTimeTest) {
  ignition::transport::Node node;

  auto server_pub = node.Advertise<smartmouse::msgs::ServerControl>(TopicNames::kServerControl);

  Server server;
  server.Connect();
  smartmouse::msgs::ServerControl unpause_msg;
  unpause_msg.set_pause(false);
  server_pub.Publish(unpause_msg);

  size_t N = 1000;
  Time start = Time::GetWallTime();
  for (size_t i = 0; i < N; i++) {
    server.Run();
  }
  Time end = Time::GetWallTime();

  // check that our total time is within one millisecond
  double actual_total_time = (end - start).Double();
  double expected_total_time = (double) (N * server.getNsOfSimPerStep()) / 1000000000.0;
  EXPECT_NEAR(actual_total_time, expected_total_time, 0.1 * N);
}

TEST(ServerTest, PIDTest) {
  ignition::transport::Node node;

  auto server_pub = node.Advertise<smartmouse::msgs::ServerControl>(TopicNames::kServerControl);

  Server server;
  server.Connect();
  smartmouse::msgs::ServerControl unpause_msg;
  unpause_msg.set_pause(false);
  server_pub.Publish(unpause_msg);

  size_t N = 1000;
  Time start = Time::GetWallTime();
  for (size_t i = 0; i < N; i++) {
    server.Run();
  }
  Time end = Time::GetWallTime();

  // check that our total time is within one millisecond
  double actual_total_time = (end - start).Double();
  double expected_total_time = (double) (N * server.getNsOfSimPerStep()) / 1000000000.0;
  EXPECT_NEAR(actual_total_time, expected_total_time, 0.1 * N);
}

TEST(RayTracingTest, DistanceToWallTest) {
  {
    ignition::math::Line2d wall({1, -0.5}, {1, 0.5});
    ignition::math::Vector2d sensor_pt(0, 0);
    ignition::math::Vector2d sensor_direction(1, 0);
    double dist = 0;
    bool intersects = RayTracing::distance_to_wall(wall, sensor_pt, sensor_direction, &dist);

    EXPECT_TRUE(intersects);
    EXPECT_EQ(dist, 1);
  }

  {
    ignition::math::Line2d wall({1, -0.5}, {1, 0.5});
    ignition::math::Vector2d sensor_pt(0, 0);
    ignition::math::Vector2d sensor_direction(-1, 0);
    double dist = 123;
    bool intersects = RayTracing::distance_to_wall(wall, sensor_pt, sensor_direction, &dist);

    EXPECT_FALSE(intersects);
    EXPECT_EQ(dist, 123);
  }

  {
    ignition::math::Line2d wall({0, 1}, {2, 3});
    ignition::math::Vector2d sensor_pt(2, 1);
    ignition::math::Vector2d sensor_direction(-1, 1);
    double dist;
    bool intersects = RayTracing::distance_to_wall(wall, sensor_pt, sensor_direction, &dist);

    EXPECT_TRUE(intersects);
    EXPECT_NEAR(dist, 1.414213, 1e-6);
  }

  {
    ignition::math::Line2d wall({0, 0}, {0, 0.18});
    ignition::math::Vector2d sensor_pt(0.09, 0.09);
    ignition::math::Vector2d sensor_direction(-1, 0);
    double dist;
    bool intersects = RayTracing::distance_to_wall(wall, sensor_pt, sensor_direction, &dist);

    EXPECT_TRUE(intersects);
    EXPECT_EQ(dist, 0.09);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
