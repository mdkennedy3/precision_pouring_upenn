#Classification#
This code was written by [Monroe Kennedy III](mailto:monroe.kennedy3@gmail.com) and Tyler Larkworky:

- **generate\_classification.py**: performs analysis and classifies volume curves using legendre polynomials. Classification is performed on normalized Legendre coefficients, and the unsupervised classification method is Guassian Mixture Model. 

##How to generate data figures ##

+ `-p` will print the figures which consist of: 

- normed\_coefficients\_for\_analysis\_for\_clustering\_3D.png: here the coefficients are represnted in 3D where the axis are labeled by color and matching coeffients
- normed\_coefficients\_for\_analysis\_for\_clustering.png: here the coefficients are represented normalized on a single plot where all coefficients share the y axis, and the data set is the x axis. lines connect the coefficients, a possibly better way would be to have ceofficients on x axis and connect for a single data set (tyler code does this and plots PCA of coefficients found from kmeans (same as GMM without covariance information)
- volume\_curves\_decreasing.png: numerical and polynomial data for V(theta) curve decreasing
- volume\_curves\_increasing\_fit.png: numerical and polynomial data for V(theta) curve increasing (this is obtained by differentiating, taking the negative and integrating the decreasing function). 
- volume\_derivative\_curves\_increasing.png: This is the derivative function

##How to store the classification data##
+ `-s` will save the data in a classification directory in this package

Formatting for this saved file: 

- The writing formatting is:

```
import cPickle as pickle
with open('data.p', 'wb') as fp:
  pickle.dump(data,fp)
```

- The reading formatting is:

```
with open('data.p', 'rb') as fp:
  data = pickle.load(fp)
```

***

The data structure has the following dictionary components: 

- ['clusters']: dict
     - ['clusters_obj']: obj: mixture.GaussianMixture (mixture object for fitting for volume functions)
     - ['cluster_center_row_major']: nxm matrix with rows having cluster center coefficients (these coefficients are for decreasing volume function)
     - ['cluster_cov']: list of np.array of covariances for each cluster center
     - ['clusters_inc']: list of dictionary for increasing cluster coefficients
          - ['cluster_series_v_inc']: numpy polynomial series object for increasing volume functions for gmm classification centers
          - ['cluster_series_intercept']: The theta intercept value for each respective cluster






