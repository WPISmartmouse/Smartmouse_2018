# SmartMouse2017


WPI CollabLab Smartmouse Team code for exploring and solving the maze.


## Programming the robot

Our build system is platformio. Install it first. To install platform IO. simply use `pip install platformio`. You may need to prepend `sudo -H` if you're on linux.

### Building Instructions (no simulation)

    mkdir .build
    cd .build
    cmake ..
    make

## The simulator

### Installing (On ubuntu 16.04+)

go to [http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) and follow the installation tutorials. You will need to install both gazebo8 and libgazebo8-dev

    sudo apt-get install gazebo8 libgazebo8-dev cmake protobuf-compiler libncurses5-dev

Test by running `gazebo --verbose`, it should open gazebo and print out version information.

Clone this repo, which contains the gazebo models and plugins.


#### Building Simulation

    mkdir .build
    cd .build
    cmake -DBUILD_SIM=ON ..
    make

#### Running Simulation

Assuming you've installed gazebo correctly and built with simuation, cd to the root of the project `Smartmouse_2017`.

    source sim/setup.sh
    gazebo --verbose sim/gzmaze.world


If you see errors, fix them before moving on. Then in another terminal...

    ./.build/SimSolve

Press enter. Watch the mouse go!
