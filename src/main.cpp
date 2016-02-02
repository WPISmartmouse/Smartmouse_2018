/** \mainpage SmartMouse V4 Index Page
 *
 * \section intro_sec Introduction
 *
 * All function documentation is done in the header files. To see notes for each function, click on the header file it belongs to.
 *
 * If there are any issues/bugs, use the issue tracker on the github repository.
 * 	[Issue tracker] (https://github.com/WPISmartmouse/SmartMouse2015_Maze/issues)
 *
 * \section run_sec_linux Building & Running (Linux)
 *
 * to make, run
 *
 *      make;./bin/explore
 *
 * you can make more debug information print out by making with the ARGS argument
 *
 *      make ARGS=-DDEBUG_PATH #this will show the mouse in the maze as it goes. press enter to step forward
 *      make ARGS=-DDEBUG #minimal debug info
 *      make ARGS=-DDBEUG_FULL #a whole ton of debug information
 *
 * \section install_sec_linux Installation (Linux)
 *
 * \subsection step1_linux Clone the repository
 *
 *      git clone https://github.com/WPISmartmouse/SmartMouse2015_Maze/
 *
 * \section install_sec_mac Installation (Mac)
 *
 * \subsection step1_mac Figure it out yourself
 *
 * \section install_sec_win Installation (Windows)
 *
 * \subsection step1_win No.
 *
 */
#ifndef EMBED

#include "maze.h"
#include "maze_io.h"
#include "solvers.h"
#include <errno.h>

int main(int argc, char* argv[]){

	char *filename;
	if (argc >= 2){
		filename = argv[1];
	}
	else {
		filename = "mazes/16x16_3.mz";
	}

	FILE *f = fopen(filename,"r");

	if (f == NULL){
		printf("error opening maze file %s %s\n",filename,strerror(errno));
		return EXIT_FAILURE;
	}

	printf("using maze file %s\n",filename);
	Maze *maze = read_from_file(f);

	left_hand_follow(maze);

	free_maze(maze);
	fclose(f);

	return EXIT_SUCCESS;
}

#endif
