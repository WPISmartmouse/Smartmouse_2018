#include <sim/lib/SimMouse.h>
#include <simulator/lib/common/TopicNames.h>

int main(int argc, const char **argv) {
  auto *timer = new SimTimer();
  Command::setTimerImplementation(timer);

  ignition::transport::Node node;
  bool success = node.Subscribe(TopicNames::kWorldStatistics, &SimTimer::worldStatsCallback, timer);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kWorldStatistics);
    return EXIT_FAILURE;
  }

  while (!timer->isTimeReady()) ;

  while (true) {
    std::cout << timer->programTimeMs() << "\n";
    usleep(1000000);
  }
}

