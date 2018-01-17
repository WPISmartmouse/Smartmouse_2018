#!/usr/bin/env python3

import argparse
import numpy as np
import csv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('infile', help='filename')
    args = parser.parse_args()

    reader = csv.reader(open(args.infile, 'r'))

    x = {}
    for line in reader:
        key = line[0]
        t = float(line[1])
        if key not in x:
            x[key] = []
        x[key].append(t)

    print("Key, Average time (micros)")
    for key, times in x.items():
        print(key, np.mean(times))


if __name__ == '__main__':
    main()
