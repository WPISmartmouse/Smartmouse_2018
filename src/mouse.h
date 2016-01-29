#ifndef MOUSE_H
#define MOUSE_H

#include "maze.h"
#include <stdlib.h>

/** \brief depresents a mouse
 * don't ever change the row/col of a mouse directly. This prevents it from working on the real robot
 * use forward and turn_to_face to move the mouse around. Once those functions work on the real robot it will port over fluidly
 */
struct _MOUSE{
	int row;
	int col;
	Direction dir;
};

typedef struct _MOUSE Mouse;

/** \biref allocates and initializes a mouse
 * be sure to free this mouse once you're done with it
 * @return the mouse
*/
Mouse *create_mouse();

/** takes the given character and moves in that direction 
 * @param mouse the mouse to move with
 * @param dir_char the direciton do drive: 'N', 'E', 'S', or 'W'
 */
void execute_command(Mouse *mouse, char dir_char);

/** base functions the emulate mouse movement calls

*/
void forward(Mouse *mouse);

/**update row and col numbers given based on dir*/
void update_pos(Direction dir, int *row, int *col);

/** literally just sets mouse position. pretty useless, but eventually will cause physical mouse to turn */
void turn_to_face(Mouse *mouse,Direction d);

/** translate a Dir (integer 0=N 1=E 2=S 3=W) into the character representation*/
char dir_to_char(Direction dir);

/** the obvious opposite of dir to char **/
Direction char_to_dir(char c); 

/** returns length 4 array of bools. must be freed */
bool *sense(Maze *maze, Mouse *mouse);

#endif