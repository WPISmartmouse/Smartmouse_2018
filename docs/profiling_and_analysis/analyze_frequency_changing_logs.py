#! /usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_csv", help="csv log file to process")

    args = parser.parse_args()

    np.set_printoptions(suppress=True, precision=3)

    data = np.genfromtxt(args.log_csv, delimiter=",", skip_header=True)

    data = data.reshape((9, 3, -1, 5))
    left = data[:, 0, :, 4]
    right = data[:, 0, :, 3]
    print(left)

    #plt.plot(diff)
    #plt.title("effect of +1 in analogWrite")
    #plt.show()


if __name__ == '__main__':
    main()
