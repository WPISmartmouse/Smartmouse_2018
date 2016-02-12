#pragma once

#include "Direction.h"
#include <stdlib.h>

/** \brief depresents a mouse
 * don't ever change the row/col of a mouse directly. This prevents it from working on the real robot
 * use forward and turn_to_face to move the mouse around. Once those functions work on the real robot it will port over fluidly
 */
class Mouse {

  public:
    /** \brief return the current column.
     * Guaranteed to be between 0 and MAZE_SIZE
     * \return current column
     */
    static int getCol();

    /** \brief return the current row.
     * Guaranteed to be between 0 and MAZE_SIZE
     * \return current row
     */
    static int getRow();

    /** \brief return the current direction.
     * \return current direction
     */
    static Direction getDir();

    /** base functions the emulate mouse movement calls
     * returns -1 if you ran into a wall
     * returns -2 if the mouses's direction is messed up
    */
    static int forward();

    /** \brief is the mouse at the center of the maze? */
    static bool atCenter();

    /** literally just sets mouse position. pretty useless, but eventually will cause physical mouse to turn */
    static void turn_to_face(Direction d);
    static void turn_to_face(char c);

  private:

    static int row, col;
    static Direction dir;
};
