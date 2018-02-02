# SmartMouse2017


WPI CollabLab Smartmouse Team code for exploring and solving the maze.
This project currently only supports Ubuntu Linux 16.04 or Fedora 26. Use windows and other linux versions at your own risk.


## Installing dependencies

Our build system is CMake 3.9.2. Install it first. Install teensy_loader_cli so you can upload. https://github.com/PaulStoffregen/teensy_loader_cli

Install the arduino ide (version 1.8.1) to `/opt`.  Next, install teensyduino.
https://www.pjrc.com/teensy/td_download.html

We recommend we using the CLion IDE.

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

### Dependencies

You need cmake 3.9.2, protobuf, ignition-transport 3.\*, and Qt 5.9.\*

**For Ubuntu 16.04**

    sudo apt-get install libprotobuf-dev libignition-transport libignition-transport-dev

Qt can be install from the Qt website.

**For Fedora 26**

    sudo dnf install ignition-transport-devel protobuf protobuf-devel

#### Building Simulation

    mkdir .sim_build
    cd .sim_build
    cmake -DBUILD_SIM=ON ..
    make

#### Running Simulation

TODO

### Running Tests

    cd .sim_build
    ./common_tests
    ./sim/simulator/sim_tests
    cd console
    ./console_tests

### Windows instructions

This is super experimental, and I don't want to support it, and it only works for the real robot not simulation.

 1. Install CMake
 1. Install arduino ide 1.8.4
 1. Install teensyduino
 1. Install CLion and clone our repoistory. Open our repository in CLion
 1. try to build. Good luck...
