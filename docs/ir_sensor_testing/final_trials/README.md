# Procedure for calibrating IR sensors

## For each sensor

 - Place the correct angle block in the last slot of the jig and start the measure_analog_curves program
 - Record the data into a csv with minicom -C #_E.csv where E is whichever sensor letter you're testing
 - Repeat 3 times, naming the log files 0_*.csv, 1_*.csv, and 2_*.csv

## Run the python program to fit exponentials to each sensor

    ../compute_calibration_curves.py *.csv

## Copy the output into your code

Copy into RobotConfig.h

## Calibration command

Turn on the robot, place it in the maze against the calibration block, and hit the "play" button. This will cause the robot to run the Calibration command. It will take a reading on each sensor, and given the known true distance to the wall, compute the necessary calibration offset to apply to each sensor.


