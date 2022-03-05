#!/usr/bin/env python
# This is the server for the ROS classification service

from pouring_unknown_geom.srv import *
from analyze import classify
from curve_fitting import fit_curve
from config import data_location
import numpy as np
import rospy
import argparse
import os

def handle_classify_container(req):
	# Fit points, classify, shift, and return new coefficients

	first_ang = min(req.angles)
	args = argparse.Namespace()
	args.derivative = 1
	args.service = True
	args.scaled = False
	args.normalized = True
	coefs = fit_curve(req.angles, req.volumes, args=args, domain=[first_ang, max(req.angles) + 0.5])
	os.chdir(data_location + '/data')
	coefs = classify(coefs, dom=[first_ang, max(req.angles) + 0.5])

	# shift curve down
	poly = np.polynomial.legendre.Legendre(coefs, domain=[first_ang, max(req.angles) + 0.5])
	y_shift = poly(first_ang)
	std_poly_coefs = poly.convert(kind=np.polynomial.polynomial.Polynomial).coef.tolist()

	# shift down
	std_poly_coefs[0] = std_poly_coefs[0] - y_shift
	new_poly = np.polynomial.polynomial.Polynomial(std_poly_coefs)

	# recast as Legendre coefficients
	coefs = np.polynomial.legendre.Legendre.cast(new_poly, domain=[first_ang, max(req.angles) + 0.05]).coef.tolist()

	return {'coefs': coefs}


def classify_container_server():
	# Run server

	rospy.init_node('classify_container_server')
	s = rospy.Service('classify_container', ClassifyContainer, handle_classify_container)
	print("===================CLASSIFIER READY===================")
	rospy.spin()

if __name__ == "__main__":
	classify_container_server()