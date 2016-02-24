#pragma once

#ifndef EMBED
  #include <fstream>
#endif
#include "Mouse.h"
#include "Direction.h"
#include "SensorReading.h"
#include "AbstractMaze.h"

class KnownMaze : public AbstractMaze {
#ifdef CONSOLE
  public: KnownMaze(std::fstream& fs);
#endif

    /* \brief check the wall surrounding you
     * \return 0 normally, OUT_OF_BOUND
     */
  public: SensorReading sense();

#ifdef SIM
  public: static void sense_callback(ConstGzStringPtr &msg);
  private: static bool walls[4];
#endif

  public: bool is_mouse_blocked();
  public: bool is_mouse_blocked(Direction dir);

  public: KnownMaze();

};
