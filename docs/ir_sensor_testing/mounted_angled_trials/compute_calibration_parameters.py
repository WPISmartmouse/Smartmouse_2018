#!/usr/bin/env python3

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize

def sensor_model(x, a, b, c):
    return a - np.maximum(x - c, 0) ** b

def main():
    """ This is for when we do all the sensors mounted and in the maze """
    #np.warnings.simplefilter("ignore", optimize.OptimizeWarning)
    #np.warnings.simplefilter("ignore", RuntimeWarning)
    np.set_printoptions(suppress=True, precision=6)

    parser = argparse.ArgumentParser()
    parser.add_argument("right_side", help="csv file output from logging serial")
    parser.add_argument("left_side", help="csv file output from logging serial")
    parser.add_argument("front", help="csv file output from logging serial")
    parser.add_argument("--no-plot", help="skip plotting", action="store_true")

    args = parser.parse_args()

    sensors = 7
    increment = 5
    max_dist = 90
    min_dist = 10
    intervals = int((max_dist + increment - min_dist) / increment)
    distances = np.arange(max_dist + increment, min_dist, -increment) * 0.001
    samples = 10
    colors = { 'A': 'r', 'B': 'b', 'D': 'g', 'E': 'y', 'F': 'm', 'G': 'k', 'H': 'sienna'}

    # will contain sensor data for G, D, and H
    right_data = np.genfromtxt(args.right_side, delimiter=', ')[:, 4:7]

    # will contain sensor data for B, A, and F
    left_data = np.genfromtxt(args.left_side, delimiter=', ')[:, 0:3]

    # will contain sensor data for E
    front_data = np.genfromtxt(args.front, delimiter=', ')[:, 3]
    front_data = np.expand_dims(front_data, 1)

    right_data = right_data.reshape((intervals, samples, right_data.shape[1]))
    left_data = left_data.reshape((intervals, samples, left_data.shape[1]))
    front_data = front_data.reshape((intervals, samples, front_data.shape[1]))

    # Take the average of the 10 samples
    right_data = right_data.mean(axis=1)
    left_data = left_data.mean(axis=1)
    front_data = front_data.mean(axis=1)

    # combine into one data structure
    data_map = {
            "G": right_data[:, 0],
            "D": right_data[:, 1],
            "H": right_data[:, 2],
            "B": left_data[:, 0],
            "A": left_data[:, 1],
            "F": left_data[:, 2],
            "E": front_data[:, 0],
            }

    # Fit a curve for each sensor
    predicted_distances = {}
    params = {}
    bounds=([-np.inf, -np.inf, -np.inf], [np.inf, 1, np.inf])
    for letter, adc_values in data_map.items():
        p, _ = optimize.curve_fit(sensor_model, adc_values, distances, bounds=bounds, maxfev=10000)
        predicted_distances[letter] = sensor_model(adc_values, *p)
        params[letter] = p


    # Each column in these three matrices represents one sensors average values over the different distances
    if not args.no_plot:
        plt.figure()
        for (letter, adc_values), predicted_distance in zip(data_map.items(), predicted_distances.values()):
            plt.plot(adc_values, distances, label=letter, color=colors[letter])
            plt.plot(adc_values, predicted_distance, label=letter, linestyle='--', color=colors[letter])
        plt.xlabel("ADC value")
        plt.ylabel("distance (meters)")
        plt.legend()
        plt.show()

    return


if __name__ == '__main__':
    main()
