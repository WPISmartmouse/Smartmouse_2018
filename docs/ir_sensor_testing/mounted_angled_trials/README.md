# Procedure for calibrating IR sensors

## mount all the sensors on the robot

 - using the laser cut calibration blocks, place the robot at known distances from the real maze wall (10mm up to 90mm)
 - Record the data into a csv with minicom -C left.csv
 - repeat for left.csv, right.csv, and front.csv

## Run the python program to fit exponentials to each sensor

    ../compute_calibration_parameters_combined.py right.csv left.csv front.csv

This program will read in and average each 10-sample for each distance for each sensor. It then fits this data to the correct distance in meters. To do this, we need to know the distance to the wall for each sensor. This is the hypotenous of a right triangle between the sensor and wall, where the sin side of the triangle is the sidewase distance (Y-axis of robot) to the wall. This is equal to the calibration block width plus some sensor offset within the robot (6mm for back, 10mm for front, 14mm for geralds, 10mm for the front).

Calibration Distances (N is the width of the block being used, units are meters and degrees):

Back L/R:   sin(85)/(0.006 + N)
Front L/R:  sin(70)/(0.010 + N)
Gerald L/R: sin(60)/(0.014 + N)
Front:      10 + N

## Copy the output into your code

Copy into RobotConfig.h

## Calibration command

Turn on the robot, place it in the maze against the calibration block, and hit the "play" button. This will cause the robot to run the Calibration command. It will take a reading on each sensor, and given the known true distance to the wall, compute the necessary calibration offset to apply to each sensor.

