# SmartMouse2017
[![Build Status](https://travis-ci.org/WPISmartmouse/2016_Solvers.svg?branch=master)](https://travis-ci.org/WPISmartmouse/2016_Solvers)


WPI CollabLab Smartmouse Team code for exploring and solving the maze.


## Building instructions

    mkdir build
    cmake ..
    make


#### If you want simulation, use this

    mkdir build
    cmake -DBUILD_SIM=ON ..
    make

## The simulator

### Installing (On ubuntu 16.04+)

go to [http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) and follow the installation tutorials. You will need to install both gazebo8 and libgazebo8-dev

    sudo apt-get install gazebo8 libgazebo8-dev cmake
    
Test by running `gazebo --verbose`, it should open gazebo

Clone this repo, which contains the gazebo models and plugins.


Assuming you've installed gazebo correctly and built with simuation...

    source sim/setup.sh
    gazebo --verbose gzmaze/gzmaze.world


If you see errors, fix them before moving on. Then in another terminal...

    ./build/SimSolve

Press enter. Watch the mouse go!
