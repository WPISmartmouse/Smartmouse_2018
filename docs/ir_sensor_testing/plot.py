#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser("plot a log file from analog_test.cpp")
    parser.add_argument("--logs", help="csv file output form logging serial", nargs="*", required=True)
    parser.add_argument("--columns", help="number of column to plot (0 indexed)", nargs="*", type=int, required=True)
    parser.add_argument("--samples-per-interval", '-n', help="number of readings at each distance", type=int, default=10)
    parser.add_argument("--number-of-distances", '-d', help="number of distances", type=int, default=17)
    parser.add_argument("--spacing", '-s', help="spacing between each interval in meters", type=float, default=0.005)

    args = parser.parse_args()

    if len(args.logs) != len(args.columns):
        print("you must provide one and only one column from every log")
        return

    # load data
    S = len(args.logs)
    N = args.samples_per_interval
    D = args.number_of_distances
    distances = np.arange(D, 0, -1) * args.spacing
    data = np.ndarray((len(args.logs), D, N))
    for i, (column, log) in enumerate(zip(args.columns, args.logs)):
        x = np.genfromtxt(log, delimiter=', ')[:, column]
        x = x.reshape((D, N))
        data[i] = x

    means = data.mean(axis=2)
    errors = np.stack((data.max(axis=2) - means, means - data.min(axis=2)), axis=1)

    # compute the mean for each interval

    plt.figure()
    for d, yerr, log in zip(data, errors, args.logs):
        plt.scatter(distances, d.mean(axis=1), label=log, s=4)
        plt.errorbar(x=distances, y=d.mean(axis=1), yerr=yerr, linewidth=1)

    plt.legend()
    plt.xlabel("sample")
    plt.ylabel("analog read value")
    plt.show()


if __name__ == '__main__':
    main()
