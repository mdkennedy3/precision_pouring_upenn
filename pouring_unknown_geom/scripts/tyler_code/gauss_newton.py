#! /usr/bin/env python

import numpy as np

DOMAIN=[0.0,1.5]

# def polynomial(coefs, x):
#     ret = 0
#     for i in range(0, len(coefs)):
#         ret += coefs[i] * (x ** i)
#     return ret

def polynomial(coefs, x):
    leg = np.polynomial.legendre.Legendre(coefs, domain=DOMAIN)
    return leg(x)

# def jacobian(xn, angles):
#     ret = np.empty([len(angles), len(xn)])
#     for i in range(0, len(angles)):
#         for j in range(0, len(xn)):
#             ret[i,j] = angles[i] ** j
#     return ret

def jacobian(xn, angles):
    ret = np.empty([len(angles), len(xn)])
    for i in range(0, len(angles)):
        for j in range(0, len(xn)):
            ret[i,j] = np.polynomial.legendre.Legendre.basis(j, domain=DOMAIN)(angles[i])
    return ret

def residuals(coefs, angles, volumes):
    ret = []
    for i in range(0, len(angles)):
        # print(polynomial(coefs, angles[i]))
        ret.append(volumes[i] - polynomial(coefs, angles[i]))
    return ret



# Credit to Basil L. Contovounesios for this code
def solve(angles, volumes, x0, tol = 1e-10, maxits = 500, eps = 1e-3):
    """Gauss-Newton algorithm for solving nonlinear least squares problems.
    Parameters
    ----------
    sys : Dataset
        Class providing residuals() and jacobian() functions. The former should
        evaluate the residuals of a nonlinear system for a given set of
        parameters. The latter should evaluate the Jacobian matrix of said
        system for the same parameters.
    x0 : tuple, list or ndarray
        Initial guesses or starting estimates for the system.
    tol : float
        Tolerance threshold. The problem is considered solved when this value
        becomes smaller than the magnitude of the correction vector.
        Defaults to 1e-10.
    maxits : int
        Maximum number of iterations of the algorithm to perform.
        Defaults to 256.
    Return
    ------
    sol : ndarray
        Resultant values.
    its : int
        Number of iterations performed.
    Note
    ----
    Uses numpy.linalg.pinv() in place of similar functions from scipy, both
    because it was found to be faster and to eliminate the extra dependency.
    """
    dx = np.ones(len(x0))   # Correction vector
    xn = np.array(x0)       # Approximation of solution

    i = 0
    while (i < maxits) and (dx[dx > tol].size > 0):
        # correction = pinv(jacobian) . residual vector
        dx  = np.dot(np.linalg.pinv(jacobian(xn, angles)), residuals(xn, angles, volumes)) * eps
        # print(dx)
        xn += dx            # x_{n + 1} = x_n + dx_n
        i  += 1

    # calculate sum of squared errors
    # print(residuals(xn, angles, volumes)[:100])
    sse = (sum([x ** 2 for x in residuals(xn, angles, volumes)]) / len(angles)) ** 0.5

    # print(sum([abs(y) for y in residuals(xn, angles, volumes)]) / len(angles))

    return xn, i, sse