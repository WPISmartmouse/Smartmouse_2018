#!/usr/bin/env python3

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize

def sensor_model(x, a, b, c):
    return a - pow(x - c, b)

def main():
    np.warnings.simplefilter("ignore", optimize.OptimizeWarning)
    np.warnings.simplefilter("ignore", RuntimeWarning)
    np.set_printoptions(suppress=True, precision=6)

    parser = argparse.ArgumentParser("plot a log file from analog_test.cpp")
    parser.add_argument("logs", help="csv file output form logging serial", nargs="+")
    parser.add_argument("--no-plot", help="skip plotting", action="store_true")
    parser.add_argument("--samples-per-interval", '-n', help="number of readings at each distance", type=int, default=10)
    parser.add_argument("--number-of-distances", '-d', help="number of distances", type=int, default=17)
    parser.add_argument("--sensors",  help="number of sensors", type=int, default=7)
    parser.add_argument("--start",  help="skip this many intervals at the beginning", type=int, default=0)
    parser.add_argument("--end", help="skip this many intervals at the end", type=int, default=1)
    parser.add_argument("--spacing", '-s', help="spacing between each interval in meters", type=float, default=0.005)

    args = parser.parse_args()

    if args.end == 0:
        print("end cannot be zero. use 1 instead")
        return

    def clip(x):
        return x[args.start:-args.end]

    # based on our calibration block
    calibration_distances = {'B': 0.0542078, 'A': 0.0595125, 'F': 0.0876708, 'E': 0.0830000, 'G': 0.0876917, 'D': 0.0595197, 'H': 0.0542047}

    # load data
    MAX_DIST = 0.2
    MIN_DIST = 0.01
    S = args.sensors
    N = args.samples_per_interval
    D = args.number_of_distances
    distances = np.arange(D, 0, -1) * args.spacing
    distances[0] = MAX_DIST
    columns = {"B": 0, "A": 1, "F": 2, "E": 3, "G": 4, "D": 5, "H": 6}
    name_map = {"B": "back_left", "A": "front_left", "F": "gerald_left", "E": "front", "G": "gerald_right", "D": "front_right", "H": "back_right"}

    letter_data_map = {}
    for i, log in enumerate(args.logs):
        letter = os.path.basename(log).strip(".csv")
        if letter not in letter_data_map:
            letter_data_map[letter] = []
        data = np.genfromtxt(log, delimiter=', ')[:, columns[letter]]
        data = data.reshape((D, N))
        mean = data.mean(axis=1)
        letter_data_map[letter].append(mean)

    letters = list(letter_data_map.keys())

    # compute average over various trials
    means = np.ndarray((S, D))
    for i, data in enumerate(letter_data_map.values()):
        data = np.array(data)
        mean = data.mean(axis=0)
        means[i] = mean

    # fit our model Y=a*X^b+c for each sensor
    params = np.ndarray((S, 3))
    model_errors = np.ndarray((S, D - args.end - args.start))
    model_predictions = np.ndarray((S, D - args.end - args.start))
    markdown_table_output = "|sensor|a|b|c|\n|------|-|-|-|"
    generated_code_output = ""
    json_output = ""
    for i, (letter, m) in enumerate(zip(letter_data_map.keys(), means)):
        # we ignore the first data point here because it doesn't have a proper distance, it's infinitly far
        # we also ignore the last three points where shit starts to go down
        m = clip(m)
        d = clip(distances)
        p, _ = optimize.curve_fit(sensor_model, m, d, maxfev=100000)
        model_prediction = sensor_model(m, *p)
        model_error = model_prediction - d

        markdown_table_output += "|{:s}|{:0.6f}|{:0.6f}|{:0.6f}|\n".format(letter, *p)
        generated_code_output += "smartmouse::ir::ModelParams"
        generated_code_output += " {:s}_model{{{:0.6f}, {:0.6f}, {:0.6f}, {:0.6f}}};\n" .format(name_map[letter], *p, calibration_distances[letter])
        json_output += name_map[letter] + "\n"
        json_output += "\"a\": {:0.6f},\n\"b\": {:0.6f},\n\"c\": {:0.6f}\n".format(*p)

        params[i] = p
        model_predictions[i] = model_prediction
        model_errors[i] = model_error

    print(markdown_table_output)
    print(generated_code_output)
    print(json_output)

    print("Average Model Parameters (these suck, so beware)")
    print(params.mean(axis=0))

    if not args.no_plot:
        colors = { 'A': 'r', 'B': 'b', 'D': 'g', 'E': 'y', 'F': 'm', 'G': 'k', 'H': 'sienna'}
        plt.figure()
        plt.plot([distances[args.start], distances[-args.end]], [0, 0], label='zero', linestyle='--')
        for letter, model_error in zip(letters, model_errors):
            plt.plot(clip(distances), model_error, label=letter)
        plt.title("Modeling Error")
        plt.xlabel("distance")
        plt.ylabel("error (meters)")
        plt.legend()

        plt.figure()
        for m, predictions, letter in zip(means, model_predictions, letters):
            plt.plot(m, distances, label=letter, linewidth=1, c=colors[letter])
            plt.plot(clip(m), predictions, label=letter + " model", alpha=0.7, linestyle='--', c=colors[letter])
        plt.title("Averaged Data")
        plt.xlabel("ADC Value 0-8192")
        plt.ylabel("Distance (meters)")
        plt.legend()

        plt.show()


if __name__ == '__main__':
    main()
