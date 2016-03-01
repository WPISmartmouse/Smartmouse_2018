#pragma once

#ifdef CONSOLE
  #include <fstream>
#endif
#ifdef SIM
  #include <mutex>
  #include <condition_variable>
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
  private: static constexpr float WALL_DIST = 0.125;
  public: static void sense_callback(ConstLaserScanStampedPtr &msg);
  private: static bool walls[4];
  private: static std::condition_variable sense_cond;
  private: static std::mutex sense_mutex;
#endif

  public: bool is_mouse_blocked();
  public: bool is_mouse_blocked(Direction dir);

  public: KnownMaze();

};
