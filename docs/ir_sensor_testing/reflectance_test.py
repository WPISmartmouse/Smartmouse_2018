#!/usr/bin/env python3

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize

def sensor_model(x, a, b):
    return a - pow(x, b)

def main():
    np.warnings.simplefilter("ignore", optimize.OptimizeWarning)
    np.set_printoptions(suppress=True, precision=6)

    parser = argparse.ArgumentParser("plot a log file from analog_test.cpp")
    parser.add_argument("logs", help="csv file output form logging serial", nargs="+")
    parser.add_argument("--no-plot", help="skip plotting", action="store_true")
    parser.add_argument("--samples-per-interval", '-n', help="number of readings at each distance", type=int, default=10)
    parser.add_argument("--number-of-distances", '-d', help="number of distances", type=int, default=7)
    parser.add_argument("--spacing", '-s', help="spacing between each interval in meters", type=float, default=0.01)

    args = parser.parse_args()

    # load data
    S = 7
    N = args.samples_per_interval
    D = args.number_of_distances
    distances = np.arange(D, 0, -1) * args.spacing + 0.01
    columns = {"B": 0, "A": 1, "F": 2, "E": 3, "G": 4, "D": 5, "H": 6}

    letter_data_map = {}
    for i, log in enumerate(args.logs):
        letter = os.path.basename(log).strip(".csv")
        data = np.genfromtxt(log, delimiter=', ')[:, columns[letter]]
        data = data.reshape((D, 2, N))
        mean = data.mean(axis=2)
        if letter in letter_data_map:
            print(letter, "already in map. overwriting.")
        letter_data_map[letter] = mean


    for letter, data in letter_data_map.items():
        white_p, _ = optimize.curve_fit(sensor_model, data[:, 0], distances, maxfev=100000)
        unpainted_p, _ = optimize.curve_fit(sensor_model, data[:, 1], distances, maxfev=100000)
        print(white_p, unpainted_p)

    if not args.no_plot:
        plt.figure()
        for letter, data in letter_data_map.items():
            plt.plot(data[:, 0], distances, label=letter + " white", linewidth=1, c='k')
            plt.plot(data[:, 1], distances, label=letter + " unpainted", linewidth=1, c='r')
        plt.title("Averaged Data")
        plt.xlabel("ADC Value 0-8192")
        plt.ylabel("Distance (meters)")
        plt.legend()

        plt.show()


if __name__ == '__main__':
    main()
