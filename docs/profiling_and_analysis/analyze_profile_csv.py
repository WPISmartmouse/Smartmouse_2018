#!/usr/bin/env python3

import argparse
import numpy as np
import csv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('infile', help='filename')
    parser.add_argument('--outfile', help='filename')
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
    if args.outfile:
        outfile = open(args.outfile, 'w')
        outfile.write("Key, Average time (micros)\n")
    for key, times in x.items():
        mean = np.mean(times)
        print(key, mean)
        if args.outfile:
            outfile.write("%s, %f\n"%(key, mean))


if __name__ == '__main__':
    main()
