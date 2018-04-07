#!/usr/bin/env python3

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

def main():
    np.set_printoptions(suppress=True, precision=6)

    parser = argparse.ArgumentParser("plot a log file from analog_test.cpp")
    parser.add_argument("log", help="csv file output with row/col/yaw")

    args = parser.parse_args()

    x = np.genfromtxt(args.log)
    yaw = -x[:, 11] + np.pi/2

    plt.figure()
    c = cm.rainbow(np.linspace(0, 1, x.shape[0]))
    plt.scatter(x[:, 9], x[:,10], c=c)
    plt.quiver(x[:, 9], x[:,10], np.cos(yaw), np.sin(yaw), scale=50)
    plt.title("Position")
    plt.xlabel("X (cells)")
    plt.ylabel("Y (cells)")
    plt.axis('square')
    plt.show()


if __name__ == '__main__':
    main()

