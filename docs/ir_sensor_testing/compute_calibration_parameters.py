#!/usr/bin/env python3

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize

def clip(x):
    return x[1:-2]

def sensor_model(x, a, b):
    return -a*pow(x, b) + 180

def bigger_sensor_model(x, a, b, c):
    return -a*pow(x, b) + c

def weighting_model():
    return np.array([1, 1, 1, 0.95, 0.8, 0.5, 0.3, 0.3, 0.5, 0.8, 0.95, 1, 1, 1])


def main():
    np.warnings.simplefilter("ignore", optimize.OptimizeWarning)
    np.set_printoptions(suppress=True, precision=6)

    parser = argparse.ArgumentParser("plot a log file from analog_test.cpp")
    parser.add_argument("--logs", help="csv file output form logging serial", nargs="*", required=True)
    parser.add_argument("--no-plot", help="skip plotting", action="store_true")
    parser.add_argument("--samples-per-interval", '-n', help="number of readings at each distance", type=int, default=10)
    parser.add_argument("--number-of-distances", '-d', help="number of distances", type=int, default=17)
    parser.add_argument("--spacing", '-s', help="spacing between each interval in meters", type=float, default=0.005)

    args = parser.parse_args()

    # load data
    MAX_DIST = 0.1
    MIN_DIST = 0.01
    S = 7
    N = args.samples_per_interval
    D = args.number_of_distances
    distances = np.arange(D, 0, -1) * args.spacing
    distances[0] = MAX_DIST
    columns = {"B": 0, "A": 1, "F": 2, "E": 3, "G": 4, "D": 5, "H": 6}

    letter_data_map = {}
    for i, log in enumerate(args.logs):
        letter = os.path.basename(log).strip(".csv")
        if letter not in letter_data_map:
            letter_data_map[letter] = []
        data = np.genfromtxt(log, delimiter=', ')[:, columns[letter]]
        data = data.reshape((D, N))
        mean = data.mean(axis=1)
        letter_data_map[letter].append(mean)

    # compute average over various trials
    means = np.ndarray((S, D))
    for i, data in enumerate(letter_data_map.values()):
        data = np.array(data)
        mean = data.mean(axis=0)
        means[i] = mean

    # fit our model Y=a*X^b+c for each sensor
    params = np.ndarray((S, 2))
    model_errors = np.ndarray((S, D-3))
    model_predictions = np.ndarray((S, D-3))
    print("|sensor|a|b|")
    print("|------|-|-|")
    for i, (letter, m) in enumerate(zip(letter_data_map.keys(), means)):
        # we ignore the first data point here because it doesn't have a proper distance, it's infinitly far
        # we also ignore the last three points where shit starts to go down
        m = clip(m)
        d = clip(distances)
        weights = weighting_model()
        p, _ = optimize.curve_fit(sensor_model, m, d, sigma=weights, maxfev=100000)
        model_prediction = sensor_model(m, *p)
        model_error = model_prediction - d
        print("|{:s}|{:0.6f}|{:0.6f}|".format(letter, *p))
        params[i] = p
        model_predictions[i] = model_prediction
        model_errors[i] = model_error

    if not args.no_plot:
        colors = { 'A': 'r', 'B': 'b', 'D': 'g', 'E': 'y', 'F': 'm', 'G': 'k', 'H': 'sienna'}
        plt.figure()
        plt.plot([distances[1], distances[-3]], [0, 0], label='zero', linestyle='--')
        for model_error, log in zip(model_errors, args.logs):
            plt.plot(clip(distances), model_error, label=log)
        plt.title("Modeling Error")
        plt.xlabel("distance")
        plt.ylabel("error (meters)")
        plt.legend()

        plt.figure()
        for m, predictions, log in zip(means, model_predictions, args.logs):
            letter = os.path.basename(log).strip(".csv")
            plt.plot(m, distances, label=letter, linewidth=1, c=colors[letter])
            plt.plot(clip(m), predictions, label=log + " model", alpha=0.7, linestyle='--', c=colors[letter])
        plt.title("Averaged Data")
        plt.xlabel("ADC Value 0-8192")
        plt.ylabel("Distance (meters)")
        plt.legend()

        plt.show()


if __name__ == '__main__':
    main()
