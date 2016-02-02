#include "mouse.h"

Mouse::Mouse() : row(0), col(0), dir(Direction::S)
 {}

void Mouse::execute_command(char dir_char){
	Direction dir = char_to_dir(dir_char);
	turn_to_face(dir);
	forward();
}

void Mouse::forward(){
	update_pos();
	if (row >= MAZE_SIZE || row < 0 || col >= MAZE_SIZE || col < 0){
		printf("OH SHIT I HIT A WALL!\n");
	}
}

void Mouse::update_pos(){
	switch(dir){
		case Direction::N:
      row--;
      break;
		case Direction::E:
      col++;
      break;
		case Direction::S:
      row++;
      break;
		case Direction::W:
      col--;
      break;
    default:
      break;
  }
}

void Mouse::turn_to_face(Direction d){
	if (dir != d){
		dir = d;
	}
	//in reality this will turn the physical mouse
}

Direction Mouse::char_to_dir(char c){
	switch(c){
    case 'N':return Direction::N;
    case 'S':return Direction::S;
    case 'E':return Direction::E;
    case 'W':return Direction::W;
    default: return Direction::INVALID;
	}
}

/**true means a wall exists**/
void Mouse::sense(Maze *maze, bool *walls){
	bool *w = walls;
	int i;
	Node *n;
  maze->get_node(&n,row,col);
	for (i=0;i<4;i++){
		*(w++) = (n->neighbors[i] == NULL);
	}
}
