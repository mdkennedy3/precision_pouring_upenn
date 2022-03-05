# curve_fitting.py
# A library of functions that process pouring data from files


import json
import numpy as np
import os
from scipy.optimize import curve_fit, minimize

POLYNOMIAL_ORDER = 9


def process_file(dir, file, MAX_VOLUME, last_ang=np.pi, angles=[], volumes=[]):
    # Process the data from a single file, and append it to lists of the angles and volumes processed so far

    data = json.load(open(dir + '/' + file))
    angs = data[0]
    vols = data[1]
    p_angs = []
    p_vols = []
    next_ang = 0.0


    # In some cases, the initial volume of the container was not set correctly, so the data has a nonzero final volume
    # This corrects the problem by shifting the volume data accordingly
    if vols[-1] != 0.0:
        for i in range(0, len(vols)):
            vols[i] -= vols[-1]


    # Now, iterate over the data from the file. But, only store the lowest volume at each angle, stop storing data after
    # the first 0 volume is encountered     
    ignore_zeros = False
    for i in range(0, len(angs) - 1):

        if vols[i] == 0.0:
            if not ignore_zeros:
                ignore_zeros = True
                p_angs.append(angs[i])
                p_vols.append(vols[i])
                if angs[i] > last_ang:
                    # Keep track of the first angle at which all of the fluid has flowed out
                    last_ang = angs[i]

        elif round(angs[i], 3) >= round(next_ang, 3) and angs[i] < angs[i+1] and (vols[i] < vols[0] or vols[i] == MAX_VOLUME):
            p_angs.append(angs[i])
            p_vols.append(vols[i])

        if round(angs[i+1], 3) > round(next_ang, 3):
            next_ang += 0.05

    # If the first processed angle is not zero for some reason, add in an artificial (0, MAX_VOLUME) data point
    if p_angs[0] != 0.0:
        p_angs.insert(0, p_angs[0] - round((p_angs[1] - p_angs[0]),2))
        p_vols.insert(0, vols[0])

    # For a processed set of data from a single file, each angle should be associated with exactly one volume.
    # This iteration ensures that an angle is not repeated.
    for i in range(0, len(p_angs) - 1):
        if i + 1 < len(p_angs) and p_angs[i] == p_angs[i + 1]:
            p_vols[i] = (p_vols[i] + p_vols[i+1]) / 2.
            p_vols.pop(i+1)
            p_angs.pop(i+1)

    #Append the end
    # p_angs.append(np.pi)
    # p_vols.append(0.01)

    # Append the processed data
    angles += p_angs
    volumes += p_vols
    return angles, volumes, last_ang


def fit_curve(angles, volumes, fn=None, args=None, domain=None, poly_order = POLYNOMIAL_ORDER):
    # Using sequential least squares programming, fit a 9th-degree, nondecreasing polynomial curve to the data
    # RETURNS: coefficients of polynomial (in Legendre basis scaled 

    last_ang = max(angles)
    if not domain:
        domain = [0.0, last_ang]
    
    def func(x, coefs):
        ret = 0
        for i in range(0, len(coefs)):
            ret += coefs[i] * (x ** i)
        return ret

    def funczero(coefs):
        if getattr(args, "service", False):
            return func(min(angles), coefs)
        return coefs[0]


    def objective(coefs):
        return ((func(np.array(angles), coefs) - volumes) ** 2).sum()

    # bounds = [(-0.1, 0.1), *list(zip(np.full(POLYNOMIAL_ORDER, -2000), np.full(POLYNOMIAL_ORDER, 2000)))]

    def con_1st_deriv(coefs):
        ret = 0
        for i in range(1, len(coefs)):
            ret += i * coefs[i] * (np.array(angles) ** (i -1))
        return ret

    cons = (dict(type="ineq", fun=con_1st_deriv), dict(type="eq", fun=funczero))


    # print(fn)

    x0 = [0.0] + list(np.full(poly_order , 0.0))
    # if args.scaled:
    #     x0 = [0.0] + list(np.full(POLYNOMIAL_ORDER , 1e-2))



    if fn == '005':
        tol = 0.1
        x0 = [0.0] + list(np.full(poly_order , 1e-2))  #just added this as it wasn't working on desktop
        # tol = 0.1
    elif fn == '016':
        tol = 0.001
    elif fn == '002':
        tol = 0.1
        x0 = [0.0] + list(np.full(poly_order , 1e-2))  #just added this as it wasn't working on desktop
    elif fn == '004':
        tol = 0.1
        x0 = [0.0] + list(np.full(poly_order , 1e-2))  #just added this as it wasn't working on desktop
    elif fn == '015':
        # tol = 0.01
        # x0 = [0.0] + list(np.full(poly_order , 1e-2))
        tol = 0.1
        x0 = [0.0] + list(np.full(poly_order , 1e-2))  #just added this as it wasn't working on desktop
    elif fn == '017':
        tol = 0.1
        x0 = [0.0] + list(np.full(poly_order , 1e-2))  #just added this as it wasn't working on desktop
    elif fn == '021':
        tol = 0.1
        x0 = [0.0] + list(np.full(poly_order , 1e-2))  #just added this as it wasn't working on desktop
    elif fn == '022':
        tol = 0.1
        x0 = [0.0] + list(np.full(poly_order , 1e-1))  #just added this as it wasn't working on desktop
    elif fn == '023':
        tol = 0.1
        x0 = [0.0] + list(np.full(poly_order , 1e-1))  #just added this as it wasn't working on desktop
    elif fn == '020':
        tol = 0.1

        ## CHANGE TO x0 = [0.0] + list(np.full(POLYNOMIAL_ORDER, 0.0)) if on KUKA
        ## CHANGE TO x0 = [0.0] + list(np.full(POLYNOMIAL_ORDER , 1e-3)) otherwise
        x0 = [0.0] + list(np.full(poly_order , 1e-3))
    else:
        tol = 0.01

    
    res = minimize(objective, x0=x0, method='SLSQP', constraints=cons, options=dict(maxiter=10000, ftol=tol))

    if res.success:
        pars = res.x
        # print(list(pars))  #uncomment to see the coefficients
        z_poly = np.polynomial.Polynomial(pars)
    else:
        print(fn, 'Failed', res.message)
        return

    #this is window useful for finding points by casting scale into domain vs changing domain which requires change in basis (and this numpy library does allow quick basis change)
    # coefs = np.polynomial.legendre.Legendre.cast(z_poly, domain=domain).deriv(m=args.derivative).coef.tolist()
    # coefs = np.polynomial.legendre.Legendre.cast(z_poly, window=domain).deriv(m=args.derivative).coef.tolist()
    coefs = np.polynomial.legendre.Legendre.cast(z_poly, window=domain, domain=domain).deriv(m=args.derivative).coef.tolist()

    if args.normalized:
        norm = np.linalg.norm(coefs)
        for j in range(0, len(coefs)):
            coefs[j] = coefs[j] / norm

    return coefs       


def process_directory(dir, args=None):
    # Process a container directory, which contains all the data for a single container
    # RETURNS: (angles, volumes) tuple representing the processed data for the container

    if 'INFO.json' not in os.listdir(dir):
        print("FATAL ERROR: INFO.json not found in directory {}".format(os.getcwd()))
        return

    # INFO.json stores the maximum volume of the container
    MAX_VOLUME = json.load(open( dir + '/INFO.json'))["max_volume"]
    angles = []
    volumes = []
    last_ang = 0

    for fn in os.listdir(dir):
        if fn != 'INFO.json' and fn[-1] != "~":
            nangles, nvolumes, nlast_ang = process_file(dir, fn, MAX_VOLUME, last_ang=last_ang, angles=angles, volumes=volumes)
            angles += nangles
            volumes += volumes
            last_ang = nlast_ang
    angles.insert(0, 0.0)
    volumes.insert(0, MAX_VOLUME)

    if not getattr(args, 'inverted', False):
        for i in range(0, len(volumes)):
            volumes[i] = MAX_VOLUME - volumes[i]

    if getattr(args, 'scaled', False):
        for i in range(0, len(volumes)):
            volumes[i] = (volumes[i] / MAX_VOLUME) * 100

    return angles, volumes
