#!/usr/bin/env python
# Provides functionality for analyzing pouring data and plotting the clusters

import json
import numpy as np 
import matplotlib.pyplot as plt
from sklearn import svm, preprocessing
from sklearn.cluster import KMeans
from scipy.optimize import minimize
from sklearn.decomposition import PCA
import os
import sys
import argparse
import platform
import gauss_newton
from linecolors import linecolors
from curve_fitting import process_file, process_directory, fit_curve
from config import data_location


# old classifications
cylinders = ['002', '003', '007', '008', '009', '012', '013', '014', '017', '018', '020']
prisms = ['001', '004', '006', '010', '011', '005', '021', '022', '023']
cones = ['015', '016', '019', '008']

neck = ['005', '015', '017', '001', '016', '019']
wide_mouth_med_size = ['003', '009', '010', '014', '020']
med_mouth_tall_size = ['002', '004', '023']
med_mouth_small_size = ['007', '006', '012', '021']
med_mouth_med_size = ['022', '008']

thin_mouth = ['001', '005', '015', '016', '017', '019']
med_mouth = ['002', '004', '006', '007', '008', '022']
wide_mouth = ['003', '009', '010', '012', '020', '021', '023', '014']


# Don't include these since they have incomplete data
outliers = ["000", "011", "013", "018", "019"]

# global variables
data = {}
angles = []
volumes = []
MAX_VOLUME = 500.0
POLYNOMIAL_ORDER = 9
DOMAIN = [0.0, 1.0]
last_ang = 0.0



def cluster(args, data):
    # Cluster the data vectors using PCA and K-means

    pca = PCA(n_components=2)
    feature_vectors = np.array(list(data.values()))
    print(feature_vectors)

    pca.fit(feature_vectors)
    print("Explained variance ratio:", pca.explained_variance_ratio_, "\n")
    transformed_vectors = pca.transform(feature_vectors)


    labeled_data = {}
    kmeans = KMeans(n_clusters=5, random_state=0).fit(transformed_vectors)

    for i in range(0, len(kmeans.labels_)):
        if labeled_data.get(kmeans.labels_[i]):
            labeled_data[kmeans.labels_[i]].append(list(data.keys())[i])
        else:
            labeled_data[kmeans.labels_[i]] = [list(data.keys())[i]]
    print("LABALED DATA:", labeled_data, "\n")

    print("Components:", pca.components_, "\n")


    # Plot data points
    ids = list(data.keys())
    fig, ax = plt.subplots()
    plt.plot(transformed_vectors[:,0], transformed_vectors[:, 1], 'bo')
    for i in range(0, len(transformed_vectors)):
        ax.annotate(ids[i], (transformed_vectors[i,0], transformed_vectors[i,1]))

    plt.show()


def classify(inital_coefficients, dom=[0.0, 1.65]):
    # Classify a container given the coefficients of a curve fitted to its initial data
    # Inputs: initial coefficients, domain to scale Legendre

    global args
    pca = PCA(n_components=2)

    args = argparse.Namespace()
    args.derivative = 0
    args.normalized = False

    for fn in os.listdir('.'):
        if os.path.isdir(fn) and fn != "misc" and fn != ".git" and fn not in outliers:
            angles, volumes = process_directory(fn, args=args)
            dump_data(fn, angles, volumes, dom=dom)

    coefs_list = list(data.values())
    for i in range(0, len(coefs_list)):
        coefs_list[i] = np.polynomial.legendre.Legendre(coefs_list[i], domain=dom).deriv().coef.tolist()

    norm = np.linalg.norm(coefs_list)
    for j in range(0, len(coefs_list)):
        coefs_list[j] = coefs_list[j] / norm

    feature_vectors = np.array(coefs_list)

    pca.fit(feature_vectors)
    print("Explained variance:", pca.explained_variance_ratio_)
    transformed_vectors = pca.transform(feature_vectors)


    labeled_data = {}    
    kmeans = KMeans(n_clusters=5, random_state=0).fit(transformed_vectors)

    class_id = kmeans.predict(pca.transform(inital_coefficients))[0]
    print("Class:", class_id)

    for i in range(0, len(kmeans.labels_)):
        if labeled_data.get(kmeans.labels_[i]):
            labeled_data[kmeans.labels_[i]].append(list(data.keys())[i])
        else:
            labeled_data[kmeans.labels_[i]] = [list(data.keys())[i]]

    sum = np.zeros(10)
    for container in labeled_data[class_id]:
        sum += np.array(data[container])
    avg = [x / len(labeled_data[class_id]) for x in sum]

    return avg



def dump_data(fn, angles, volumes, dom=[0.0, 1.65]):
    # Fit data to curves and store in global data dict

    global data
    coefs = fit_curve(angles, volumes, fn=fn, args=args, domain=dom)
    
    data[fn] = coefs


def plot_coefs(args, data):
    # Plot the Legendre coefficients

    i = 0
    for d in data:
        c = linecolors[i]
        line, = plt.plot(range(0, (POLYNOMIAL_ORDER + 1 if  not args.derivative else POLYNOMIAL_ORDER)), data[d], color=c, linestyle='-', linewidth=1.25)
        line.set_label(d)
        i += 1

    leg = plt.legend()
    for legobj in leg.legendHandles:
        legobj.set_linewidth(3.0)

    plt.xlabel("Legendre coefficient number")
    plt.ylabel("Coefficient value")
    plt.show()


def main():
    global args

    # Set up arg parser

    parser = argparse.ArgumentParser(description="Analyze and plot clusters in pouring data")
    # parser.add_argument("-s", "--scaled", action="store_true")
    parser.add_argument("-D", "--derivative", help="perform analysis on derivative coefficients", action="count", default=0)
    parser.add_argument("-c", "--colors", action="store_true")
    parser.add_argument("-n", "--normalized", action="store_true")
    subparsers = parser.add_subparsers()

    parser_clusters = subparsers.add_parser("cluster")
    parser_clusters.set_defaults(func=cluster)
    

    # parser_clusters = subparsers.add_parser("cluster_alt")
    # parser_clusters.set_defaults(func=cluster_alt)

    # parser_classify = subparsers.add_parser("classify")
    # parser_classify.set_defaults(func=classify_old)

    parser_plot = subparsers.add_parser("plot")
    parser_plot.set_defaults(func=plot_coefs)

    args = parser.parse_args()

    for fn in os.listdir('.'):
        if os.path.isdir(fn) and fn != "misc" and fn != ".git" and fn not in outliers:
            angles, volumes = process_directory(fn, args=args)
            dump_data(fn, angles, volumes)

    # json.dump(data, open('coefficients.json', 'w'))

    args.func(args, data)




if __name__ == '__main__':
    # CD into directory contianing data dir
    os.chdir(os.path.join(os.pardir, os.pardir, os.pardir, os.pardir))
    if platform.system() == 'Linux':
        os.chdir(data_location)
    print(os.getcwd())
    if 'data' not in os.listdir('.'):
        print("ERROR: data directory not found in root project directory!")
        sys.exit()
    os.chdir('data')
    main()