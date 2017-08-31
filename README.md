# SmartMouse2017


WPI CollabLab Smartmouse Team code for exploring and solving the maze.
This project currently only supports Ubuntu Linux 16.04. Use windows and other linux versions at your own risk.


## Installing dependencies

Our build system is CMake. Install it first. Install teensy_loader_cli so you can upload. https://github.com/PaulStoffregen/teensy_loader_cli

Install the arduino ide (version 1.8.1) to `/opt`.  Next, install teensyduino.
https://www.pjrc.com/teensy/td_download.html

### Building Instructions (no simulation)

    mkdir .build
    cd .build
    cmake ..
    make

### Uploading to the robot

After connecting the usb cable, `cd .build` and run the following:

    teensy_loader_cli --mcu=MK66FX1M0 -w -s -v bin/main.elf.hex

Obviously this requires teensy_loader_cli


## The simulator

#### Building Simulation

    mkdir .sim_build
    cd .sim_build
    cmake -DBUILD_SIM=ON ..
    make

#### Running Simulation

TODO
