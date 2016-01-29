#include "maze.h"
#include "maze_io.h"

int main(int argc, char *argv[]){
  char *maze_file = malloc(100 * sizeof(char));
  if (argc >= 2) {
    maze_file = argv[1];
    printf("Printing maze %s\n", maze_file);
  }
  else {
    maze_file = "mazes/16x16.mz";
  }

  if (maze_file != NULL){
    FILE *f = fopen(maze_file,"r");

    if (f > 0){
      Maze *maze = read_from_file(f);
      print_maze(maze);
      return EXIT_SUCCESS;
    }
  }
	return EXIT_FAILURE;
}
