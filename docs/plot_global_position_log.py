#!/usr/bin/env python3

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt

def main():
    np.set_printoptions(suppress=True, precision=6)

    parser = argparse.ArgumentParser("plot a log file from analog_test.cpp")
    parser.add_argument("log", help="csv file output with row/col/yaw")

    args = parser.parse_args()

    x = np.genfromtxt(args.log, delimiter=',')
    yaw = -x[:, 2] + np.pi/2

    plt.figure()
    plt.scatter(x[:, 0], x[:,1])
    plt.quiver(x[:, 0], x[:,1], np.cos(yaw), np.sin(yaw), scale=10)
    plt.title("Position")
    plt.xlabel("X (cells)")
    plt.ylabel("Y (cells)")
    plt.axis('square')
    plt.show()


if __name__ == '__main__':
    main()

