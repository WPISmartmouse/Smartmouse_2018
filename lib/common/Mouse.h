#pragma once

#include "Direction.h"
#include "AbstractMaze.h"

/** \brief depresents a mouse
 * don't ever change the row/col of a mouse directly. This prevents it from working on the real robot
 * use forward and turnToFace to move the mouse around. Once those functions work on the real robot it will port over fluidly
 */
class Mouse {

  public:

    Mouse(AbstractMaze *maze);

    Mouse(AbstractMaze *maze, int starting_row, int starting_col);

    /** \brief return the current column.
     * Guaranteed to be between 0 and MAZE_SIZE
     * \return current column
     */
    int getCol();

    /** \brief return the current row.
     * Guaranteed to be between 0 and MAZE_SIZE
     * \return current row
     */
    int getRow();

    /** \brief return the current direction.
     * \return current direction
     */
    Direction getDir();

    /** check if we're in bounds */
    bool inBounds();

    /** \brief is the mouse at the center of the maze? */
    bool atCenter();

    void turnToFace(char c);

    void mark_mouse_position_visited();

    /** \brief get the node that the mouse is currently on
     */
    Node *get_mouse_node();

    void internalTurnToFace(Direction dir);
    void internalForward();

    /** prints a maze with a mouse in it
    * @param mouse the mouse
    * @param maze the maze
    */
    void print_maze_mouse();

    virtual SensorReading sense() = 0;
    bool is_mouse_blocked();
    bool is_mouse_blocked(Direction dir);

    AbstractMaze *maze;

    static constexpr float ROT_TOLERANCE = 0.01;

  protected:

    int row, col;
    Direction dir;
};
