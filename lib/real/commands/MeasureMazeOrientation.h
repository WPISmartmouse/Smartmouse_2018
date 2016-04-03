#pragma once

#include "CommanDuino.h"
#include "RealMouse.h"

class MeasureMazeOrientation : public Command {
  public:
    MeasureMazeOrientation();
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    const int MAX_SAMPLE_COUNT;
    float data;
    int readyToExit;
    RealMouse *mouse;
    uint32_t lastDisplayUpdate;
};
