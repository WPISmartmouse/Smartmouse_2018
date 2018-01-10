#include <iostream>

#include <ignition/transport.hh>

#include <simulator/lib/common/TopicNames.h>

void help();

int main() {

  help();

  ignition::transport::Node node;

  auto pub = node.Advertise<ignition::msgs::Vector2d>(TopicNames::kPIDSetpoints);

  usleep(1000000); // 1 s

  bool done = false;
  while (!done) {
    ignition::msgs::Vector2d speeds;

    float left_speed_cps;
    float right_speed_cps;

    scanf("%f, %f", &left_speed_cps, &right_speed_cps);

    speeds.set_x(left_speed_cps);
    speeds.set_y(right_speed_cps);

    pub.Publish(speeds);
  }
}

void help() {
  std::cout << "Usage: ./pub_speeds" << std::endl;
  std::cout << std::endl;
  std::cout << "Repeatedly enter two (can be floating point) numbers separated by one comma and one space" << std::endl;
  std::cout << "EX: 1.5, 2" << std::endl;

}
