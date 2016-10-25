# SmartMouse2015_Maze
[![Build Status](https://travis-ci.org/WPISmartmouse/2016_Solvers.svg?branch=master)](https://travis-ci.org/WPISmartmouse/2016_Solvers)


WPI CollabLab Smartmouse Team code for exploring and solving the maze.


## Building instructions

    mkdir build
    cmake ..
    make


#### If you want simultaion, use this

    mkdir build
    cmake .. -DBUILD_SIM=ON
    make

## Running the simulator

Assuming you've installed gazebo correctly and built with simuation...

    source sim/setup.sh
    gazebo --verbose gzmaze/gzmaze.world


If you see errors, fix them before moving on. Then in another terminal...

    ./build/SimSolve

Press enter. Watch the mouse go!
