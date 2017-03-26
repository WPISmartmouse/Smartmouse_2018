#pragma once

#include <common/commanduino/Command.h>
#include <real/RealMouse.h>

class MeasureMazeOrientation : public Command {
public:
  MeasureMazeOrientation();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  RealMouse *mouse;
  const int MAX_SAMPLE_COUNT;
  float data;
  bool readyToExit;
  uint32_t lastDisplayUpdate;
};
