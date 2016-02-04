#pragma once

#include "Maze.h"
#include "Direction.h"
#include <stdlib.h>

/** \brief depresents a mouse
 * don't ever change the row/col of a mouse directly. This prevents it from working on the real robot
 * use forward and turn_to_face to move the mouse around. Once those functions work on the real robot it will port over fluidly
 */
class Mouse {

  public:
    /** \biref allocates and initializes a mouse */
    Mouse();

    /** takes the given character and moves in that direction
     * @param dir_char the direciton do drive: 'N', 'E', 'S', or 'W'
     */
    void execute_command(char dir_char);

    /** base functions the emulate mouse movement calls
    */
    void forward();

    /**update row and col numbers given based on dir*/
    void update_pos();

    /** literally just sets mouse position. pretty useless, but eventually will cause physical mouse to turn */
    void turn_to_face(Direction d);

    /** the obvious opposite of dir to char **/
    Direction char_to_dir(char c);

    /** returns length 4 array of bools. must be freed */
    void sense(Maze *maze, bool *walls);

    int row;
    int col;
    Direction dir;
};
