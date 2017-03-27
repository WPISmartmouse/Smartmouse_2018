#ifdef CONSOLE

#include "ConsoleMouse.h"
#include "ConsoleMaze.h"
#include <iostream>

#ifdef __linux__
#include <unistd.h>
#else
#include "WinGetOpt.h"
#endif

int main(int argc, char* argv[]){
	int c,row=0,col=0;
	opterr = 0;
	while ((c = getopt (argc, argv, "r:c:")) != -1) {
		switch (c) {
			case 'r':
				row = atoi(optarg);
				break;
			case 'c':
				col = atoi(optarg);
				break;
			case '?':
				if (optopt == 'r' || optopt == 'c')
					fprintf (stderr, "Option -%c requires an argument.\n", optopt);
				else if (isprint (optopt))
					fprintf (stderr, "Unknown option `-%c'.\n", optopt);
				else
					fprintf (stderr,
							"Unknown option character `\\x%x'.\n",
							optopt);
				return 1;
			default:
				abort ();
		}
	}

	if (argc < 3){
		printf("USAGE: Animate maze path [-r row] [-c col]\n");
		printf("the maze is the maze file to run the path in\n");
		printf("the path argument should be a string of N, S, E, and W.\n");
		printf("row the start row, col is the starting col\n");
		exit(0);
	}

  std::string maze_file;
  std::string path;
	maze_file = std::string(argv[optind++]);
	path = std::string(argv[optind]);

  std::fstream fs;
  fs.open(maze_file, std::fstream::in);

  if (fs.good()){
    std::cout << "maze file: " << maze_file << std::endl;
    std::cout << "path: " << path << std::endl;
    std::cout << "start pos: (" << row << "," << col << ")" << std::endl;

    ConsoleMaze maze(fs);
    ConsoleMouse::inst()->seedMaze(&maze);

		int i = 0;
    while (ConsoleMouse::inst()->inBounds() && i < path.length()){
			ConsoleMouse::inst()->print_maze_mouse();
			ConsoleMouse::inst()->internalTurnToFace(char_to_dir(path.at(i++)));
			ConsoleMouse::inst()->internalForward();
      std::cin.get();
    }

    ConsoleMouse::inst()->print_maze_mouse();
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
