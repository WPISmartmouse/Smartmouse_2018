#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("log")

    args = parser.parse_args()

    x = np.genfromtxt(args.log, delimiter=', ')

    plt.figure()
    time = np.arange(0, x.shape[0]) * 1/65
    plt.plot(time, x[:, 0]*0.0145, label='left setpoint')
    plt.plot(time, x[:, 1]*0.0145, label='left measured')
    plt.plot(time, x[:, 2]*0.0145, label='right setpoint')
    plt.plot(time, x[:, 3]*0.0145, label='right measured')
    plt.ylabel("meters/second")
    plt.xlabel("time")
    plt.legend()

    plt.show()


if __name__ == '__main__':
    main()
