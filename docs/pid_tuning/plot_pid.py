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
    plt.plot(x[:, 0]*0.0145, label='setpoint')
    plt.plot(x[:, 1]*0.0145, label='measured')
    plt.ylabel("meters/second")
    plt.xlabel("time")
    plt.legend()

    plt.figure()
    plt.plot(x[:, 2])
    plt.ylabel("force")
    plt.xlabel("time")

    plt.show()


if __name__ == '__main__':
    main()
