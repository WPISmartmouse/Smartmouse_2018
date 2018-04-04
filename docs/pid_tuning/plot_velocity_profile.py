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
    plt.plot(x[:, 2], label='left force')
    plt.plot(x[:, 5], label='right force')
    plt.ylabel("force")
    plt.xlabel("time")
    plt.legend()

    plt.figure()
    plt.plot(x[:, 0]*0.0145, label='left setpoint')
    plt.plot(x[:, 1]*0.0145, label='left measured')
    plt.plot(x[:, 3]*0.0145, label='right setpoint')
    plt.plot(x[:, 4]*0.0145, label='right measured')
    plt.ylabel("meters/second")
    plt.xlabel("time")
    plt.legend()

    plt.show()


if __name__ == '__main__':
    main()
