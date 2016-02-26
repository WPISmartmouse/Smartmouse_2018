#pragma once

enum class Direction {
  N, //0
  E, //1
  S, //2
  W, //3
	Last, //4
	First = N, //0
  INVALID = -1
};

/** \brief returns the left of the given direction */
Direction left_of_dir(Direction d);

double yawDifference(Direction d1, Direction d2);

char opposite_direction(char c);

/**
 * \brief increments the direction in the order N, E, S, W, N, ...
 */
Direction operator++(Direction& dir, int);

/**
 * \brief returns the opposite direction of the input direction
 * \param d direction you want the opposite of
 * \return the opposite direction of d
 */
Direction opposite_direction(Direction d);

/** translate a Dir (integer 0=N 1=E 2=S 3=W) into the character representation*/
char dir_to_char(Direction dir);


/** translate a char into the direction representation*/
Direction char_to_dir(char c);
