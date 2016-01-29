/** maze.c implements the functions used to create, free, and manipulate maze
	@author Peter Mitrano
*/
#include "maze.h"

Maze *create_maze(){
	Maze *maze = malloc(sizeof(Maze));
	maze->solved = false;
	maze->fastest_route = malloc(PATH_SIZE*sizeof(char)); //assume really bad route--visits all squares.
	int i,j;
	for (i=0;i<MAZE_SIZE;i++){
		for (j=0;j<MAZE_SIZE;j++){
			maze->nodes[i][j] = create_node();
			maze->nodes[i][j]->row = i;
			maze->nodes[i][j]->col = j;
		}
	}
	return maze;
}

Node *create_node(){
	Node *node = malloc(sizeof(Node));
	node->known = false;
	node->weight = -1;
	node->neighbors[N]=NULL;
	node->neighbors[S]=NULL;
	node->neighbors[E]=NULL;
	node->neighbors[W]=NULL;
	return node;
}

Node *get_node(Maze *maze, int row, int col){
	if (col < 0 || col >= MAZE_SIZE || row < 0 || row >= MAZE_SIZE){
		return NULL;
	}
	return maze->nodes[row][col];
}

void update_nodes(bool *walls, Direction dir, Node *no, Node *all, Node *all_neighbor){

	Direction opposite_direction = (dir + 2) % 4;

	//if a wall exists in that direction, add a wall to no_wall_maze
	if (walls[dir]){
		//making the neighbor null represents adding a wall
		//make sure to update both the node and the node it no longer connects to!
		if (no->neighbors[dir] != NULL){
			no->neighbors[dir]->neighbors[opposite_direction] = NULL;
		}
		no->neighbors[dir] = NULL;
	}
	//if no wall exists in that direction remove a wall from all_wall_maze
	else {
		//getting the previously unconnected neighbor represents adding a wall
		//make sure to update both the node and the node it now connects to
		if (all_neighbor != NULL){
			all_neighbor->neighbors[opposite_direction] = all;
		}
		all->neighbors[dir] = all_neighbor;
	}
}

Node *maze_diff(Maze *maze1, Maze *maze2){
	Node *new_goal = NULL;
	
	int i,j,max = -1;
	for (i=0;i<MAZE_SIZE;i++){
		for (j=0;j<MAZE_SIZE;j++){

			Node *n1 = get_node(maze1,i,j);
			Node *n2 = get_node(maze2,i,j);

			//don't look at nodes you've already actually visited
			if (!n1->visited && !n2->visited){
				int count1=0,count2=0;

				Direction d;
				for (d=0;d<4;d++){
					if (n1->neighbors[d] == NULL) count1++;
					if (n2->neighbors[d] == NULL) count2++;
				}
				
				int diff = abs(count2-count1);

				if (diff > max){
					max = diff;
					//doesn't matter n1 or n2, all we're using are the row/col
					new_goal = n1;
				}
			}
		}
	}

	return new_goal;
}

bool visited(int row, int col, Direction dir, Maze *maze){
	return maze->nodes[row][col]->neighbors[dir]->known;
}

bool free_maze(Maze *maze){
	free(maze->fastest_route);
	int i,j;
	for (i=0;i<MAZE_SIZE;i++){
		for (j=0;j<MAZE_SIZE;j++){
			free(maze->nodes[i][j]);
		}
	}
	free(maze);
	return true;
}

//nothing fancy here, just convenient. might remove if it never gets more complicated
bool free_node(Node *node){
	free(node);	
	return true;
}
