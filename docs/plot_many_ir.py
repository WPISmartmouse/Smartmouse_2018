#!/usr/bin/env python3

import argparse
from math import sqrt
import os
import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple

SensorPose = namedtuple("SensorPose", ["x", "y", "angle"])

def from_sensors_to_left_wall(s1, s2, s1_dist_m, s2_dist_m):
    d, yaw = from_sensors_to_wall(s1, s2, s1_dist_m, s2_dist_m)
    return d, yaw

def from_sensors_to_right_wall(s1, s2, s1_dist_m, s2_dist_m):
    d, yaw = from_sensors_to_wall(s1, s2, s1_dist_m, s2_dist_m)
    return d, -yaw

def from_sensors_to_wall(s1, s2, s1_dist_m, s2_dist_m):
	d1x = np.cos(s1.angle) * s1_dist_m + s1.x
	d1y = np.sin(s1.angle) * s1_dist_m + s1.y
	d2x = np.cos(s2.angle) * s2_dist_m + s2.x
	d2y = np.sin(s2.angle) * s2_dist_m + s2.y
	yaw = -np.atan2(d2y - d1y, d2x - d1x)
	dist = abs(d2x * d1y - d2y * d1x) / sqrt(pow(d2y - d1y, 2) + pow(d2x - d1x, 2))
	return dist, yaw

def make_line(s, d):
    dx = np.cos(s.angle) * d + s.x
    dy = np.sin(s.angle) * d + s.y
    return [[s.x, dx], [s.y, dy]]

def main():
    np.set_printoptions(suppress=True, precision=6)

    parser = argparse.ArgumentParser()
    parser.add_argument("log")

    args = parser.parse_args()

    many_ranges = np.genfromtxt(args.log)

    FRONT_ANALOG_ANGLE = 1.2217
    BACK_ANALOG_ANGLE = 1.48353
    FRONT_SIDE_ANALOG_X = 0.033
    FRONT_SIDE_ANALOG_Y = 0.026
    BACK_SIDE_ANALOG_X = -0.026
    BACK_SIDE_ANALOG_Y = 0.03
    FRONT_ANALOG_X = 0.045
    GERALD_X = 0.040
    GERALD_Y = 0.022
    GERALD_ANGLE = 1.0472

    back_left = SensorPose(BACK_SIDE_ANALOG_X, BACK_SIDE_ANALOG_Y, BACK_ANALOG_ANGLE)
    front_left = SensorPose(FRONT_SIDE_ANALOG_X, FRONT_SIDE_ANALOG_Y, FRONT_ANALOG_ANGLE)
    gerald_left = SensorPose(GERALD_X, GERALD_Y, GERALD_ANGLE)
    front = SensorPose(0.045, 0, 0)
    back_right = SensorPose(BACK_SIDE_ANALOG_X, -BACK_SIDE_ANALOG_Y, -BACK_ANALOG_ANGLE)
    front_right = SensorPose(FRONT_SIDE_ANALOG_X, -FRONT_SIDE_ANALOG_Y, -FRONT_ANALOG_ANGLE)
    gerald_right = SensorPose(GERALD_X, -GERALD_Y, -GERALD_ANGLE)

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    lines = [make_line(back_left, many_ranges[0, 0]),
             make_line(front_left, many_ranges[0, 1]),
             make_line(gerald_left, many_ranges[0, 2]),
             make_line(front, many_ranges[0, 3]),
             make_line(gerald_right, many_ranges[0, 4]),
             make_line(front_right, many_ranges[0, 5]),
             make_line(back_right, many_ranges[0, 6])]

    line_plots = []
    for line in lines:
        plot, = ax.plot(line[0], line[1], c='k')
        line_plots.append(plot)

    i = 0;
    while i < len(many_ranges):
        ranges = many_ranges[i]
        print(i, ranges)
        lines = [make_line(back_left, ranges[0]),
                 make_line(front_left, ranges[1]),
                 make_line(gerald_left, ranges[2]),
                 make_line(front, ranges[3]),
                 make_line(gerald_right, ranges[4]),
                 make_line(front_right, ranges[5]),
                 make_line(back_right, ranges[6])]
        for plot, line in zip(line_plots, lines):
            plot.set_xdata(line[0])
            plot.set_ydata(line[1])
        plt.axis('square')
        ax.set_ylim([-0.2, 0.2])
        fig.canvas.draw()
        c = input()
        if c == 'a' and i > 0:
            i -= 1
        else:
            i += 1




if __name__ == '__main__':
    main()

