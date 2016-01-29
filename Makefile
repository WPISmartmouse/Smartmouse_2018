all: explore read_and_print

explore: build/explore.o build/solvers.o build/maze_io.o build/maze.o build/mouse.o
	gcc -g build/explore.o build/solvers.o build/maze_io.o build/maze.o build/mouse.o -o build/explore

read_and_print: build/read_and_print.o build/maze_io.o build/maze.o
	gcc -g build/read_and_print.o build/maze_io.o build/maze.o -o build/read_and_print

build/mouse.o: src/mouse.c src/mouse.h
	gcc -g -Wall -c src/mouse.c -o build/mouse.o

build/explore.o: src/explore.c
	gcc -g -Wall -c src/explore.c -o build/explore.o

build/solvers.o: src/solvers.c src/solvers.h
	gcc -g -Wall ${ARGS} -c src/solvers.c -o build/solvers.o

build/read_and_print.o: src/read_and_print.c
	gcc -g -Wall -c src/read_and_print.c -o build/read_and_print.o

build/maze_io.o: src/maze_io.c src/maze_io.h
	gcc -g -Wall -c src/maze_io.c -o build/maze_io.o

build/maze.o: src/maze.c src/maze.h
	gcc -g -Wall -c src/maze.c -o build/maze.o

clean:
	rm build/*
