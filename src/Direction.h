#pragma once

enum class Direction {
  N,
  E,
  S,
  W,
	Last,
	First = N,
  INVALID = -1
};

/** \brief returns the left of the given direction */
Direction left_of_dir(Direction d);


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

