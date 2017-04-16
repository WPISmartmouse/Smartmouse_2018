#pragma once

#include <math.h>
#include "Direction.h"
#include "AbstractMaze.h"
#include "Pose.h"

typedef struct {
  double front_left_analog;
  double front_right_analog;
  double back_left_analog;
  double back_right_analog;
  double front_analog;
} RangeData;

/** \brief depresents a mouse
 * don't ever change the row/col of a mouse directly. This prevents it from
 * working on the real robot
 * use forward and turnToFace to move the mouse around. Once those functions
 * work on the real robot it will port over fluidly
 */
class Mouse {

public:

  static double radToMeters(double x);

  static double meterToRad(double x);

  Mouse();

  Mouse(AbstractMaze *maze);

  Mouse(unsigned int starting_row, unsigned int starting_col);

  Mouse(AbstractMaze *maze, unsigned int starting_row, unsigned int starting_col);

  void reset();

  /** \brief return the current column.
   * Guaranteed to be between 0 and MAZE_SIZE
   * \return current column
   */
  unsigned int getCol();

  /** \brief return the current row.
   * Guaranteed to be between 0 and MAZE_SIZE
   * \return current row
   */
  unsigned int getRow();

  /** \brief return the current direction.
   * \return current direction
   */
  Direction getDir();

  /** check if we're in bounds */
  bool inBounds();

  /** \brief is the mouse at the center of the maze? */
  bool atCenter();

  void mark_mouse_position_visited();

  void internalTurnToFace(Direction dir);

  void internalForward();

  /** prints a maze with a mouse in it */
  void print_maze_mouse();

  /** creates one long string of the mouse in the maze in ascii */
  void maze_mouse_string(char *);

  virtual SensorReading checkWalls() = 0;


  /** doesn't simply read sensors. Check the internal maze structure */
  bool isWallInDirection(Direction d);

  AbstractMaze *maze;

  static constexpr double WHEEL_RAD = 0.0145;

  virtual double getColOffsetToEdge() = 0;

  virtual double getRowOffsetToEdge() = 0;

  virtual Pose getPose() = 0;

protected:
  unsigned int row, col;
  Direction dir;
};
