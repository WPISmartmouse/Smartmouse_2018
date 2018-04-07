#!/usr/bin/env python3

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize

def inverse_model(m, a, b, c):
    return pow(a - m, 1/b) + c

def sensor_model(x, a, b, c):
    return np.minimum(a - np.maximum(x - c, 0) ** b, 0.20)

def main():
    """ This is for when we do all the sensors mounted and in the maze """

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
    intervals = int((max_dist + increment - min_dist) / increment) + 1
    perpendicular_distances = np.arange(max_dist , min_dist - increment, -increment) * 0.001
    perpendicular_distances = np.concatenate(([0.20], perpendicular_distances))
    samples = 10
    colors = {'A': 'r', 'B': 'b', 'D': 'g', 'E': 'y', 'F': 'm', 'G': 'k', 'H': 'sienna'}
    variable_names = {
        'B': 'back_left',
        'A': 'front_left',
        'F': 'gerald_left',
        'E': 'front',
        'G': 'gerald_right',
        'D': 'front_right',
        'H': 'back_right'
    }
    # These are based on our calibration block.
    calibration_block_left = 0.05
    calibration_block_front = 0.09
    maze_width = 0.168
    robot_width = 0.072
    calibration_distances = {
        'B': (0.006 + calibration_block_left) / np.sin(np.deg2rad(85)),
        'A': (0.010 + calibration_block_left) / np.sin(np.deg2rad(70)),
        'F': (0.012 + calibration_block_left) / np.sin(np.deg2rad(50)),
        'E': 0.010 + calibration_block_front,
        'G': (0.012 + (maze_width - calibration_block_left - robot_width)) / np.sin(np.deg2rad(50)),
        'D': (0.010 + (maze_width - calibration_block_left - robot_width)) / np.sin(np.deg2rad(70)),
        'H': (0.006 + (maze_width - calibration_block_left - robot_width)) / np.sin(np.deg2rad(85))
    }
    hypotenuses = {
        'B': (0.006 + perpendicular_distances) / np.sin(np.deg2rad(85)),
        'A': (0.010 + perpendicular_distances) / np.sin(np.deg2rad(70)),
        'F': (0.012 + perpendicular_distances) / np.sin(np.deg2rad(50)),
        'E': 0.010 + perpendicular_distances[:-2],
        'G': (0.012 + perpendicular_distances) / np.sin(np.deg2rad(50)),
        'D': (0.010 + perpendicular_distances) / np.sin(np.deg2rad(70)),
        'H': (0.006 + perpendicular_distances) / np.sin(np.deg2rad(85))
    }

    hypotenuses['F'][0] = 0.2
    hypotenuses['G'][0] = 0.2

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
    front_data = front_data[:-2,:,:]

    # Compute Y-Errors
    right_xerr = right_data.max(axis=1) - right_data.min(axis=1)
    left_xerr = left_data.max(axis=1) - left_data.min(axis=1)
    front_xerr = front_data.max(axis=1) - front_data.min(axis=1)

    # Take the average of the 10 samples
    right_data = right_data.mean(axis=1)
    left_data = left_data.mean(axis=1)
    front_data = front_data.mean(axis=1)

    # combine into one data structure
    data_map = {
            "B": left_data[:, 0],
            "A": left_data[:, 1],
            "F": left_data[:, 2],
            "E": front_data[:, 0],
            "G": right_data[:, 0],
            "D": right_data[:, 1],
            "H": right_data[:, 2],
            }
    xerr_map = {
            "B": left_xerr[:, 0],
            "A": left_xerr[:, 1],
            "F": left_xerr[:, 2],
            "E": front_xerr[:, 0],
            "G": right_xerr[:, 0],
            "D": right_xerr[:, 1],
            "H": right_xerr[:, 2],
            }


    # Fit a curve for each sensor
    predicted_distances = {}
    params = {}
    bounds=([-np.inf, -np.inf, -np.inf], [np.inf, 1, np.inf])
    errors = {}
    for letter, adc_values in data_map.items():
        distances = hypotenuses[letter]
        p, _ = optimize.curve_fit(sensor_model, adc_values, distances, bounds=bounds, maxfev=10000)
        test_adc_values = np.arange(100, 2500, 50)
        prediction = sensor_model(test_adc_values, *p)
        predicted_distances[letter] = (test_adc_values, prediction)
        params[letter] = p
        errors[letter] = sensor_model(adc_values, *p) - distances

    print("Calibration Distances & ADC values")
    for letter, d in calibration_distances.items():
        print(letter, d, inverse_model(d, *params[letter]))

    # print generated code
    generated_code = []
    for letter, p in params.items():
        variable_name = variable_names[letter]
        fmt = "smartmouse::ir::EEPROMModel {:s}_model = {{{:.5f}, {:.5f}, {:.5f}, {:.5f}}};\n"
        generated_code.append(fmt.format(variable_name, *p, calibration_distances[letter]))

    print("".join(sorted(generated_code)))


    # plotting
    if not args.no_plot:
        plt.figure()
        for letter, adc_values in data_map.items():
            plt.plot(adc_values, errors[letter], label=letter, color=colors[letter])
        plt.xlabel("ADC value")
        plt.ylabel("error (meters)")
        plt.legend()

        plt.figure()
        for letter, adc_values in data_map.items():
            plt.errorbar(adc_values, hypotenuses[letter], xerr=xerr_map[letter], label=letter, color=colors[letter])
            px, py = predicted_distances[letter]
            plt.plot(px, py, label=letter + ' model', linestyle='--', color=colors[letter])
        plt.xlabel("ADC value")
        plt.ylabel("distance (meters)")
        plt.legend()


        plt.show()

    return


if __name__ == '__main__':
    main()
