# Precision Pouring Repo
This repo is for precision robotic pouring. The key aspect of this work is that little augmentation is required for the robot to pour fluids simliar to human counter parts.

## Directory Organization
### liquid level detection packages: 
  * liquid\_level\_detection
    * This script uses background subtraction to detect the height of fluid in the image
  * liquid\_level\_detection\_nn
    * This script runs a NN to determine the presence of distorted/occluded background, then passes the detection image to the background subtraction package to find the height of the water
### Supporting packages
  * queen\_cam\_calibration 
    * This has launch file for mvbluefox camera
  * liquid\_apriltags
    * This detects apriltags of interest in the experiment
  * tag\_generator
    * This makes the apriltags
  * pouring\_edge\_detection
    * This package begins to address how to detect the edge of containers for pouring without spilling

### Precision pouring calculations packages
  * (older) pouring\_control\_pkg 
    * (previous method for pouring with known geometry)
  * pouring\_unknown\_geom
    * This package contains the latest method of precision pouring from uknown container geometries in a single attempt

### Dependencies
  * libgp: This is for c++ gaussian process functionality: 
```bash 
git clone git@github.com:mdkennedy3/libgp.git
```

