#!/usr/bin/env python
# plot.py
# Script to plot polynomial curves for containers

import json
import matplotlib.pyplot as plt
import numpy as np
import argparse 
import sys
import platform
import os

from analyze import classify
from curve_fitting import process_file, process_directory, fit_curve
from scipy.optimize import minimize
from linecolors import linecolors
from config import data_location
import gauss_newton


# global variables
POLYNOMIAL_ORDER = 9
i = 0
angles = []
volumes = []
MAX_VOLUME = 500.0

# Don't include these because they have incomplete data
outliers = ["000", "011", "013", "018","S266","S002","S007","S038","S045","S048","S049","S051","S074","S076","S078","S080","S084","S098","S103", "S105",'S106', 'S107', 'S109', 'S110', 'S111', 'S112', 'S113', 'S126', 'S129', 'S130', 'S131', 'S133', 'S136', 'S138', 'S140', 'S141', 'S142', 'S143', 'S145', 'S146', 'S147', 'S148', 'S150', 'S158', 'S159', 'S161', 'S163', 'S165', 'S166', 'S167', 'S168', 'S169', 'S170', 'S171', 'S172', 'S173', 'S174', 'S175', 'S176', 'S177', 'S180', 'S186', 'S190', 'S191', 'S193', 'S194', 'S195', 'S197', 'S198', 'S200', 'S201', 'S202', 'S203', 'S204', 'S205', 'S206', 'S208', 'S209', 'S210', 'S211', 'S212', 'S236', 'S257', 'S259', 'S260', 'S271', 'S272', 'S275', 'S276']

# Old classification
cylinders = ['002', '003', '007', '008', '009', '012', '013', '014', '017', '018', '020']
prisms = ['001', '004', '006', '010', '011', '005', '021', '022', '023']
cones = ['015', '016', '019', '008']


def plot_curve(fn, coefs, last_ang, args=None, points=None):
    # Plot curve with the given Legendre coefficients


    global i

    leg = np.polynomial.legendre.Legendre(coefs, domain=[0.0, last_ang])

    [x, y] = leg.linspace(500, domain=[0.0, last_ang])

    if args and getattr('args', 'colors', False):
        if fn in cylinders:
            label = ' [cyl]'
            c = 'r'
        elif fn in prisms:
            label = ' [rec]'
            c = 'b'
        elif fn in cones:
            label = ' [con]'
            c = 'g'
    else:
        c = linecolors[i]


    line, = plt.plot(x, y, color=c)

    line.set_label(fn)

    if points:
        plt.plot(points["angles"], points["volumes"], 'bo')

    if fn == '007':
        print("LAST ANGLE", last_ang)

    i += 1

def plot_all():
    # Iterate over all the container subdirectories within the data directory, process the data, and plot each curve

    for fn in os.listdir('.'):
        if os.path.isdir(fn) and fn != "misc" and fn != ".git" and fn not in outliers:
            try:
                angles, volumes = process_directory(fn, args=args)
                coefs = fit_curve(angles, volumes, fn=fn, args=args)
                plot_curve(fn, coefs, max(angles), args=args)
            except:
                pass

    # leg = np.polynomial.legendre.Legendre(avg, domain=[0.0, 1.0])
    # [x_l, y_l] = leg.linspace(500, domain=[0.0,2.0])
    # plt.plot(x_l, y_l, 'k--')

    # print(g_angles[25])
    # sol, numit, sser = gauss_newton.solve(g_angles[:25], g_volumes[:25], x0)
    # # sol = x0
    # # for i in range(11, 20):
    # #     sol, numit, sser = gauss_newton.solve(g_angles[:i], g_volumes[:i], sol)
    # #     print(i)

    # print(sol, numit, sser)

    # poly = np.polynomial.legendre.Legendre(sol, domain=[0.0, 1.0])
    # [x_gn, y_gn] = poly.linspace(500, domain=[0.0, 2.0])

    # plt.plot(x_gn, y_gn, 'k--')


def plot_all_test(colors):
    global angles
    global volumes
    global last_ang

    for fn in os.listdir():
        if os.path.isdir(fn) and fn != "misc" and fn != ".git" and fn not in outliers:
            # print(fn)
            process_directory(fn)

            angles.insert(0, 0.0)
            volumes.insert(0, MAX_VOLUME)
            d_angles.insert(0, 0.0)
            d_volumes.insert(0, 0.0)

            if args.inverted:
                for i in range(0, len(volumes)):
                    volumes[i] = MAX_VOLUME - volumes[i]
                for i in range(0, len(d_volumes)):
                    d_volumes[i] *= -1

            if args.scaled:
                for i in range(0, len(volumes)):
                    volumes[i] = (volumes[i] / MAX_VOLUME) * 100

            plot_data(fn, colors)
            angles = []
            volumes = []
            last_ang = 0.0

    test_angs = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5]
    test_vols = [0.0, 0.0, 0.0, 4.0, 8.0, 12.0, 16.0, 22.0, 26.0, 32.0, 36.0]
    upbound = [1.0] + list(np.full(9, 100.0))
    lowbound = np.full(10, 0.0)
    bounds = list(zip(list(lowbound), upbound))
    print(bounds)

    def func(x, coefs):
        ret = 0
        for i in range(0, len(coefs)):
            ret += coefs[i] * (x ** i)
        return ret

    def funczero(coefs):
        return coefs[0]

    def objective(coefs):
        return ((func(np.array(test_angs), coefs) - test_vols) ** 2).sum()

    def con_1st_deriv(coefs):
        ret = 0
        for i in range(1, len(coefs)):
            ret += i * coefs[i] * (np.array(test_angs) ** (i -1))
        return ret

    cons = (dict(type="ineq", fun=con_1st_deriv), dict(type="eq", fun=funczero))
    tol = 0.00001

    res = minimize(objective, x0=np.full(POLYNOMIAL_ORDER + 1, 1.0), method='SLSQP', constraints=cons, options=dict(maxiter=10000, ftol=tol), bounds=bounds)
    if res.success:
        pars = res.x
        z_poly = np.polynomial.Polynomial(pars)
    else:
        print('Failed', res.message)
        return

    leg = np.polynomial.legendre.Legendre.cast(z_poly, domain=[0, 1.0])
    [x_l, y_l] = leg.linspace(500, domain=[0.0, 1.0])
    line2, = plt.plot(x_l, y_l, color='c', linestyle="--")

    if len(list(leg.coef)) < POLYNOMIAL_ORDER + 1:
        coefs = list(leg.coef) + list(np.full(POLYNOMIAL_ORDER + 1 - len(list(leg.coef)), 0.0))
        leg = np.polynomial.legendre.Legendre(classify(coefs), domain=[0.0, 1.0])
    else:
        leg = np.polynomial.legendre.Legendre(classify(list(leg.coef)), domain=[0.0, 1.0])

    print(leg.coef)
    [x_l, y_l] = leg.linspace(500, domain=[0.0, 1.0])
    line2, = plt.plot(x_l, y_l, color='k', linestyle="--")

    

def plot_dir(dirname):
    # Plot data for a single container

    global args
    angles, volumes = process_directory(dirname, args=args)
    coefs = fit_curve(angles, volumes, fn=dirname, args=args)
    plot_curve(dirname, coefs, max(angles), args=args, points=dict(angles=angles, volumes=volumes))


def main():
    global POLYNOMIAL_ORDER
    global args

    # Set up arg parser

    parser = argparse.ArgumentParser(description="Plot curves for single container or all containers")

    parser.add_argument("-a", "--all", help="plot all curves", action="store_true")
    # parser.add_argument("-i", "--inverted", help="plot curve of container being poured into", action="store_true")
    parser.add_argument("-d", "--dir", help="plot one container")
    # parser.add_argument("-c","--colors", action="store_true")
    parser.add_argument("-n", "--normalized", help="normalize coefficients before plotting", action="store_true")
    # parser.add_argument("-o", "--order", help="polynomial order", type=int)
    parser.add_argument("-D", "--derivative", help="plot derivative", action="count", default=0)
    args = parser.parse_args()

    if args.dir:    
        plot_dir(args.dir)
    elif args.all:
        plot_all()

    leg = plt.legend()
    for legobj in leg.legendHandles:
        legobj.set_linewidth(3.0)

    axes = plt.gca()
    axes.set_xlim([0.0, np.pi])
    plt.xlabel(r"$\theta$ (rad)")
    plt.ylabel('Volume (mL)')
    plt.show()


if __name__ == '__main__':
    if platform.system() == "Linux":
        os.chdir(data_location)
    else:
        os.chdir(os.path.join(os.pardir, os.pardir, os.pardir, os.pardir)) # move up 4 directories
    print(os.getcwd())
    if 'data' not in os.listdir('.'):
        print("ERROR: data directory not found in root project directory!")
        sys.exit()
    os.chdir('data')
    main()
