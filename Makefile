all: explore read_and_print

arduino:
	ano build --cppflags="-std=c++11 -DEMBED"

explore: src/SensorReading.h src/SensorReading.cpp src/Explore.cpp src/AbstractMaze.h src/AbstractMaze.cpp src/KnownMaze.cpp src/KnownMaze.h src/Mouse.cpp src/Mouse.h src/Solvers.cpp src/Solvers.h src/Node.cpp src/Node.h src/Direction.cpp src/Direction.h
	mkdir -p build
	g++ -std=c++11 -g -o build/explore src/Explore.cpp src/Mouse.cpp src/Solvers.cpp src/Node.cpp src/Direction.cpp src/AbstractMaze.cpp src/KnownMaze.cpp src/SensorReading.cpp

read_and_print: src/SensorReading.cpp src/SensorReading.h src/ReadAndPrint.cpp src/AbstractMaze.h src/AbstractMaze.cpp src/KnownMaze.cpp src/KnownMaze.h src/Mouse.cpp src/Mouse.h src/Solvers.cpp src/Solvers.h src/Node.cpp src/Node.h src/Direction.cpp src/Direction.h
	mkdir -p build
	g++ -std=c++11 -g -o build/read_and_print src/ReadAndPrint.cpp src/Mouse.cpp src/Solvers.cpp src/Node.cpp src/Direction.cpp src/AbstractMaze.cpp src/KnownMaze.cpp src/SensorReading.cpp

test: all test/Test.h test/Test.cpp
	mkdir -p build
	g++ -std=c++11 -g -o build/test -I src/ test/Test.cpp src/Mouse.cpp src/Solvers.cpp src/Node.cpp src/Direction.cpp src/AbstractMaze.cpp src/KnownMaze.cpp src/SensorReading.cpp

	./build/test


clean:
	rm build/*
	ano clean
