#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
    parser = argparse.ArgumentParser("plot a log file from analog_test.cpp")
    parser.add_argument("log", help="csv file output form logging serial")
    parser.add_argument("column", help="number of column to plot (0 indexed)", type=int, default=0)
    args = parser.parse_args()
    x = np.genfromtxt(args.log, delimiter=',')
    plt.plot(x[:, args.column])
    plt.xlabel("sample")
    plt.ylabel("analog read value")
    plt.show()
