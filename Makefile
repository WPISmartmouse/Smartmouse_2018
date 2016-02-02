all: explore read_and_print

arduino:
	ano build --cppflags="-std=c++11 -DARDUINO"

explore: build/explore.o build/solvers.o build/maze_io.o build/maze.o build/mouse.o
	g++ -std=c++11 -g build/explore.o build/solvers.o build/maze_io.o build/maze.o build/mouse.o -o build/explore

read_and_print: build/read_and_print.o build/maze_io.o build/maze.o
	g++ -std=c++11 -g build/read_and_print.o build/maze_io.o build/maze.o -o build/read_and_print

build/mouse.o: src/mouse.cpp src/mouse.h
	g++ -std=c++11 -g -Wall -c src/mouse.cpp -o build/mouse.o

build/explore.o: src/explore.cpp
	g++ -std=c++11 -g -Wall -c src/explore.cpp -o build/explore.o

build/solvers.o: src/solvers.cpp src/solvers.h
	g++ -std=c++11 -g -Wall ${ARGS} -c src/solvers.cpp -o build/solvers.o

build/read_and_print.o: src/read_and_print.cpp
	g++ -std=c++11 -g -Wall -c src/read_and_print.cpp -o build/read_and_print.o

build/maze_io.o: src/maze_io.cpp src/maze_io.h
	g++ -std=c++11 -g -Wall -c src/maze_io.cpp -o build/maze_io.o

build/maze.o: src/maze.cpp src/maze.h
	g++ -std=c++11 -g -Wall -c src/maze.cpp -o build/maze.o

clean:
	rm build/*
