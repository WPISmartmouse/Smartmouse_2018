#!/usr/bin/env python3

import argparse
import numpy as np
import numpy.polynomial.polynomial as poly
import matplotlib.pyplot as plt


def main():
    np.set_printoptions(suppress=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=-5)

    args = parser.parse_args()

    x = np.genfromtxt(args.log, delimiter=', ')
    vls = []
    vrs = []
    last_tl = x[0, 1]
    last_tr = x[0, 2]
    for row in x:
        tl, tr = row[1], row[2]
        def diff(t2, t1):
            d = t2 - t1
            if d > 10000:
                return d - 16384
            elif d < -10000:
                return d + 16384
            else:
                return d
        vl = diff(tl, last_tl) * 2 * np.pi / 2**14 / 0.01
        vr = -diff(tr, last_tr) * 2 * np.pi / 2**14 / 0.01
        last_tl = tl
        last_tr = tr
        vls.append(vl)
        vrs.append(vr)

    # detect the intervals
    v = np.array([vls, vrs])
    v = v.reshape((2, -1, 200))

    # compute the medians
    medians = np.median(v, axis=2)[:,args.start:args.end]

    # line of best fit
    f = np.unique(x[:,0])[args.start:args.end]
    left_coeff = poly.polyfit(medians[0], f, deg=1)
    right_coeff = poly.polyfit(medians[1], f, deg=1)
    avg_coeff = (left_coeff + right_coeff)/2

    test_f = np.linspace(f[0], f[-1], 100)
    test_model = poly.polyval(test_f, avg_coeff)

    print("Average linear model: ")
    print("{:0.6f} * regulated_setpoint_rps + {:0.6f}".format(*avg_coeff))

    if not args.no_plot:
        plt.figure()
        plt.scatter(x[:, 0], vls, label='left', s=1)
        plt.scatter(x[:, 0], vrs, label='right', s=1)
        plt.scatter(f, medians[0], label='left median')
        plt.scatter(f, medians[1], label='right median')
        plt.plot(test_f, test_model, label='right median')
        plt.ylabel("m/s")
        plt.xlabel("force (0-1024)")
        plt.legend()

        plt.show()


if __name__ == '__main__':
    main()
