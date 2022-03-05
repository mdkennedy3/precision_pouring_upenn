#Classification Object#

This scripts uses pickle to save the data needed for classification

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
         - ['opt_coeff']: **This is the optimal coefficients, filled in in realtime for a given container, this is not preset**
