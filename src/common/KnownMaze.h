#pragma once

#ifdef CONSOLE
  #include <fstream>
#endif
#ifdef SIM
  #include <mutex>
  #include <condition_variable>
  #include <gazebo/msgs/msgs.hh>
#endif
#include "Mouse.h"
#include "Direction.h"
#include "SensorReading.h"
#include "AbstractMaze.h"

class KnownMaze : public AbstractMaze {

  public:

    /* \brief check the wall surrounding you
     * \return 0 normally, OUT_OF_BOUND
     */
    SensorReading sense();

    bool is_mouse_blocked();
    bool is_mouse_blocked(Direction dir);

    KnownMaze(Mouse *mouse);

  private:
    static constexpr float WALL_DIST = 0.125;

#ifdef CONSOLE
  public: KnownMaze(std::fstream& fs, Mouse *mouse);
#endif

#ifdef SIM
  public:

    void sense_callback(ConstLaserScanStampedPtr &msg);
    std::condition_variable sense_cond;

  private:

    bool walls[4];
    std::mutex sense_mutex;
#endif

};
