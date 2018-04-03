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
    plt.scatter(x[:, 0], x[:, 1], label='left')
    plt.scatter(x[:, 0], x[:, 2], label='right')

    plt.show()


if __name__ == '__main__':
    main()
