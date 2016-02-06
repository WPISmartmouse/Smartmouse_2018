#pragma once

#ifndef EMBED
  #include <fstream>
#endif
#include "Mouse.h"
#include "SensorReading.h"
#include "AbstractMaze.h"

class KnownMaze : public AbstractMaze {
  public:
    KnownMaze(std::fstream& fs);

    /* \brief check the wall surrounding you
     * \return 0 normally, OUT_OF_BOUND
     */
    SensorReading sense();

};
