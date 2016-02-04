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

#include "Maze.h"
#include "MazeIO.h"
#include "Solvers.h"
#include <errno.h>
#include  <fstream>

int main(int argc, char* argv[]){

  std::string maze_file;
  if (argc < 2) {
    maze_file = "mazes/16x16.mz";
  }
  else {
    maze_file = std::string(argv[1]);
  }

  std::fstream fs;
  fs.open(maze_file, std::fstream::in);

  if (fs.good()){
    Maze maze(fs);
    flood_explore(maze);
    fs.close();
    return EXIT_SUCCESS;
  }
  else {
		printf("error opening maze file!\n");
    fs.close();
    return EXIT_FAILURE;
  }
}

#endif
