#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize

def clip(x):
    return x[1:-3]

def sensor_model(x, a, b, c):
    return a*pow(x, b) + c

def main():
    np.warnings.simplefilter("ignore", optimize.OptimizeWarning)
    np.set_printoptions(suppress=True, precision=6)

    parser = argparse.ArgumentParser("plot a log file from analog_test.cpp")
    parser.add_argument("--logs", help="csv file output form logging serial", nargs="*", required=True)
    parser.add_argument("--columns", help="number of column to plot (0 indexed)", nargs="*", type=int, required=True)
    parser.add_argument("--no-plot", help="skip plotting", action="store_true")
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
    distances[0] = 0.2

    data = np.ndarray((len(args.logs), D, N))
    for i, (column, log) in enumerate(zip(args.columns, args.logs)):
        x = np.genfromtxt(log, delimiter=', ')[:, column]
        x = x.reshape((D, N))
        data[i] = x

    # compute the mean and errors for each interval
    means = data.mean(axis=2)
    ranges = np.stack((data.max(axis=2) - means, means - data.min(axis=2)), axis=1)

    # fit our model Y=a*X^b+c for each sensor
    params = np.ndarray((S, 3))
    model_errors = np.ndarray((S, D-4))
    print("|sensor|a|b|c|stddev|")
    print("|------|-|-|-|------|")
    for i, m in enumerate(means):
        # we ignore the first data point here because it doesn't have a proper distance, it's infinitly far
        # we also ignore the last three points where shit starts to go down
        m = clip(m)
        d = clip(distances)
        p, _ = optimize.curve_fit(sensor_model, m, d, maxfev=10000)
        model_error = sensor_model(m, *p) - d
        print("|{:s}|{:0.6f}|{:0.6f}|{:0.6f}|".format(args.logs[i].strip(".csv"), *p))
        params[i] = p
        model_errors[i] = model_error

    # compute the "average" model
    average_model_params = params.mean(axis=0)
    average_model_errors = np.ndarray((S, D-4))
    for i, m in enumerate(means):
        m = clip(m)
        average_model_error = model_error = sensor_model(m, *average_model_params) - d
        average_model_errors[i] = average_model_error

    if not args.no_plot:
        plt.figure()
        plt.plot([distances[1], distances[-3]], [0, 0], label='zero', linestyle='--')
        for model_error, log in zip(model_errors, args.logs):
            plt.plot(clip(distances), model_error, label=log)
        plt.title("Modeling Error")
        plt.xlabel("distance")
        plt.ylabel("error (meters)")
        plt.legend()

        plt.figure()
        plt.plot([distances[1], distances[-3]], [0, 0], label='zero', linestyle='--')
        for average_model_error, log in zip(average_model_errors, args.logs):
            plt.plot(clip(distances), average_model_error, label=log)
        plt.title("Error with 'Average' Model ")
        plt.xlabel("distance")
        plt.ylabel("error (meters)")
        plt.legend()

        plt.figure()
        for d, xerr, log in zip(data, ranges, args.logs):
            plt.scatter(x=d.mean(axis=1), y=distances, label=log, s=4)
            plt.errorbar(x=d.mean(axis=1), y=distances, xerr=xerr, linewidth=1)
        plt.title("Averaged Data")
        plt.xlabel("ADC Value 0-8192")
        plt.ylabel("Distance (meters)")
        plt.legend()

        plt.show()


if __name__ == '__main__':
    main()
