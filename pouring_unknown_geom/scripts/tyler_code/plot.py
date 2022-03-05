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
outliers = ["000", "011", "013", "018"]

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
            angles, volumes = process_directory(fn, args=args)
            coefs = fit_curve(angles, volumes, fn=fn, args=args)
            plot_curve(fn, coefs, max(angles), args=args)

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

    # test_angs = [0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]
    # test_vols = [0.0, 0.0, 4.0, 6.0, 10.0, 14.0, 18.0, 22.0, 26.0, 30.0, 36.0]

    # first_ang = min(test_angs)
    # args_n = argparse.Namespace()
    # args_n.derivative = 1
    # args_n.service = True
    # args_n.scaled = False
    # args_n.normalized = True
    # coefs = fit_curve(test_angs, test_vols, args=args_n, domain=[first_ang, max(test_angs) + 0.5])
    # coefs = classify(coefs, dom=[first_ang, max(test_angs) + 0.5])

    # # shift curve down
    # poly = np.polynomial.legendre.Legendre(coefs, domain=[first_ang, max(test_angs) + 0.5])

    # [x_l, y_l] = poly.linspace(500, domain=[first_ang, max(test_angs) + 0.5])
    # plt.plot(test_angs, test_vols, 'go')
    # # plt.plot(x_l, y_l, 'c-')

    # y_shift = poly(first_ang)
    # print(y_shift)
    # std_poly_coefs = poly.convert(kind=np.polynomial.polynomial.Polynomial).coef.tolist()
    # # shift down
    # std_poly_coefs[0] = std_poly_coefs[0] - y_shift
    # new_poly = np.polynomial.polynomial.Polynomial(std_poly_coefs)

    # [x, y] = new_poly.linspace(500, domain=[0.3, 1.3])
    # plt.plot(x,y,'c-')
    # coefs = np.polynomial.legendre.Legendre.cast(new_poly, domain=[first_ang, max(test_angs) + 0.5]).coef.tolist()


    # if len(list(leg.coef)) < POLYNOMIAL_ORDER + 1:
    #     coefs = list(leg.coef) + list(np.full(POLYNOMIAL_ORDER + 1 - len(list(leg.coef)), 0.0))
    #      x0 = classify(coefs)
    # else:
    #     leg = classify(list(leg.coef))


    # x0 = [83.605712603003496, 112.19792956440011, 30.957655861302523, 7.0591023406489128, 4.2062670092995971, -0.8511904507665552, -0.35596981569520614, 0.015605585490414835, 0.0078119262183947635, 3.0544355911754642e-05]

    # leg = np.polynomial.legendre.Legendre(x0, domain=[0.0, 1.0])
    # leg = leg.convert(domain=[0.0, 1.5])
    # [x_l, y_l] = leg.linspace(500, domain=[0.0,2.0])
    # plt.plot(x_l, y_l, 'g--')

    
    # x0 = leg.coef.tolist()
    # print(x0)

    # print(g_angles[25])


    # sol, numit, sser = gauss_newton.solve(angles[:25], volumes[:25], x0)

    # # sol = x0
    # # for i in range(11, 25):
    # #     sol, numit, sser = gauss_newton.solve(angles[:i], volumes[:i], sol)
    # #     print(i)

    # print(sol, numit, sser)

    # poly = np.polynomial.legendre.Legendre(sol, domain=[0.0, 1.5])
    # [x_gn, y_gn] = poly.linspace(500, domain=[0.0, 2.0])

    # plt.plot(x_gn, y_gn, 'c--')


    # upbound = [x + abs(x * 0.4) for x in x0]
    # lowbound = [x - abs(x * 0.4) for x in x0]
    # bounds = list(zip(lowbound, upbound))
    # # print(bounds)

    # def func(x, coefs):
    #     return np.polynomial.legendre.Legendre(coefs, domain=[0.0, 1.5])(x)

    # def objective(coefs):
    #     return ((func((np.array(angles[:20])), coefs) - volumes[:20]) ** 2).sum()

    # def con_1st_deriv(coefs):
    #     return np.polynomial.legendre.Legendre(coefs, domain=[0.0,1.5]).deriv()(np.array(angles[:25]))

    # cons = (dict(type="ineq", fun=con_1st_deriv))
    # tol = 0.0001

    # res = minimize(objective, x0=x0, method='SLSQP', constraints=cons, options=dict(maxiter=10000, ftol=tol), bounds=bounds)
    # if res.success:
    #     pars = res.x
    #     print(pars)
    # else:
    #     print('Failed', res.message)
    #     return

    # leg = np.polynomial.legendre.Legendre(pars, domain=[0, 1.5])
    # [x_l, y_l] = leg.linspace(500, domain=[0.0, 2.0])
    # line2, = plt.plot(x_l, y_l, color='c', linestyle="--")

    # plt.plot(angles[19], volumes[19], 'ro')


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
