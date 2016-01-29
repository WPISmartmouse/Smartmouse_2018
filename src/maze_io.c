#include "maze_io.h"

Maze *read_from_file(FILE *f){
	//check for bad file
	if (!f) return NULL;

	Maze *maze = create_maze();

	int len = 2 * MAZE_SIZE + 3;
	char *line = malloc(len*sizeof(char));
	char *l = line;
	char *success;

	int i;
	for (i=0;i< MAZE_SIZE;i++){ //read in each line
		success = fgets(line, len + 1,f);
		l = line;

		if (success == NULL) return NULL;

		int j;
		for (j=0;j<MAZE_SIZE;j++){
			Node *n = get_node(maze,i,j);
			Node *n_west = get_node(maze,i,j-1);
			Node *n_south = get_node(maze,i+1,j);

			//make sure west node isn't null
			if (n_west){
				if (*l != '|'){
					n->neighbors[W] = n_west;
					n_west->neighbors[E] = n;
				}
			}

			l++;

			//make sure south node isn't null
			if (n_south){
				if (*l != '_'){
					n->neighbors[S] = n_south;
					n_south->neighbors[N] = n;
				}
			}

			l++;
		}
	}
	printf("\n");

	free(line);

	return maze;
}

void print_maze_mouse(Maze *maze, Mouse *mouse){
	int i,j;
	for (i=0;i<MAZE_SIZE;i++){
		char *str = malloc((MAZE_SIZE * 2 + 1) * sizeof(char));

		char *s=str;
		for (j=0;j<MAZE_SIZE;j++){
			if (get_node(maze,i,j)->neighbors[W] == NULL){
				strcpy(s++,"|");
			}
			else {
				strcpy(s++,"_");
			}

			if (mouse->row == i && mouse->col == j){
					strcpy(s++,"o");
			}
			else if (get_node(maze,i,j)->neighbors[S] == NULL){
					strcpy(s++,"_");
			}
			else {
				strcpy(s++," ");
			}
		}
		*(s++) = '|';
		*s = '\0';
		printf("%s\n",str);
		free(str);
	}
}


void print_maze(Maze *maze){
	int i,j;
	for (i=0;i<MAZE_SIZE;i++){
		char *str = malloc((MAZE_SIZE * 2 + 1) * sizeof(char));

		char *s=str;
		for (j=0;j<MAZE_SIZE;j++){
			if (get_node(maze,i,j)->neighbors[W] == NULL){
				strcpy(s++,"|");
				if (get_node(maze,i,j)->neighbors[S] == NULL){
					strcpy(s++,"_");
				}
				else {
					strcpy(s++," ");
				}
			}
			else {
				strcpy(s++,"_");
				if (get_node(maze,i,j)->neighbors[S] == NULL){
					strcpy(s++,"_");
				}
				else {
					strcpy(s++," ");
				}
			}
		}
		*(s++) = '|';
		*s = '\0';
		printf("%s\n",str);
		free(str);
	}
}

void print_neighbor_maze(Maze *maze){
	int i,j;
	for (i=0;i<MAZE_SIZE;i++){
		for (j=0;j<MAZE_SIZE;j++){
			int d;
			for (d=0;d<4;d++){
				bool wall = (maze->nodes[i][j]->neighbors[d] == NULL);
				printf("%i",wall);
			}
			printf(" ");
		}
		printf("\n");
	}	
}

void print_weight_maze(Maze *maze){
	int i,j;
	for (i=0;i<MAZE_SIZE;i++){
		for (j=0;j<MAZE_SIZE;j++){
			int w = maze->nodes[i][j]->weight;
			printf("%03d ",w);	
		}
		printf("\n");
	}
}


void print_dist_maze(Maze *maze){
	int i,j;
	for (i=0;i<MAZE_SIZE;i++){
		for (j=0;j<MAZE_SIZE;j++){
			int d = get_node(maze,i,j)->distance;
			if (d<10){
				printf("  %d ",d);
			}
			else if (d<100){
				printf(" %d ",d);
			}
			else {
				printf("%d ",d);
			}
		}
		printf("\n");
	}
}

void print_pointer_maze(Maze *maze){
	int i,j;
	for (i=0;i<MAZE_SIZE;i++){
		for (j=0;j<MAZE_SIZE;j++){
			printf("%p ", get_node(maze,i,j));
		}
		printf("\n");
	}
}
