# Pouring curve scripts #

The following scripts were written by [Tyler Larkworthy](mailto:tlarkwor@seas.upenn.edu):

- **analyze.py**: provides analysis and visualization of clusters in the pouring curves and provides the `classify` function, which is integral to the classifier service
- **classify\_container\_server.py**: implements the ROS service for classifying the container
- **classify\_srv\_test.py**: code to test the service
- **curve_fitting.py**: library of functions that fit nondecreasing polynomial curves to data and process pouring data from files
- **dump_coefficients.py**: deprecated
- **gauss_newton.py**: provides implementation of the Gauss-Newton optimization algorithm
- **gripper (shell script)**: wrapper for ROS command needed to open/close Robotiq gripper
- **linecolors.py**: stores list of many color codes that are distinct, used for plotting many curves at once
- **only_pouring.py**: simple script designed to test pouring set-up by only rotating the gripper
- **plot.py**: code to plot pouring curves with a variety of options available
- **track_angle.py**: implements a ROS node designed to work in conjunction with `iiwa_pouring_main.py` to collect the pouring data and store it in a JSON file

**iiwa\_pouring\_main** was also modified to include implementation of the actual pouring motion

## Collecting new pouring data ##

### How pouring data is stored ###

All pouring data is stored in the `data` directory, which currently is located in `\home\kuka_demo\ws_kuka\src` on the KUKA Linux machine. Within the `data` directory, each container (labeled with a 3 digit ID), has its own subdirectory, containing all of the data collected for that container.

Each new trial is stored in a JSON file, which is named with the container ID, date, and time of the data collection. The JSON file stores data in a 2D list, with the following format: `[angles, volumes]`.

**Important**: Volume data is currently collected from the perspective of the container being poured _out of_, so the maximum volume of that container must be measured before starting to collect data from a container

Also in each container's subdirectory is a file named `INFO.json`, which stores a dict with a single key/value pair noting the maximum volume of the container. For any new container, this file should be created as soon as the first trial is completed. It is essential to correctly plotting and analyzing pouring data.

### Overview of a pouring trial ###

To begin, make sure you know the maximum volume of the container. Next, make sure the KUKA arm is set-up and the GRL driver is running on both the KUKA itself and the Linux machine. Run the ROS launch file `pour_test.launch`, and then wait. The arm should move to the home position, then to the pouring position.

The terminal will prompt for the container ID, then the maximum volume. You must enter these extactly and hit `enter` after each of them. It will then prompt you to adjust the gripper and hit `enter`. Before you send the command to close the gripper, ensure the container you wish to test has been filled most (but not all) of the way. Now send the command to close the gripper, you will have 10 seconds before the gripper actually closes if you use the shell script `gripper`. Once the gripper has gripped the container, fill it up the rest of the way, until it is almost overflowing. Now, hit `enter`, and the pouring program should start.

**Important**: Make sure that the USB scale does not turn off during this time, or else the volume data will not be collected! If the scale turns off during the process of setting up the gripper, you must manually restart the node in another terminal window after turning on the scale again.

You should see the angle and volumes points plotted live as the data is collected. Wait until the rotation is complete before doing anything.

**Important**: In order for the script to properly save the data file, _you must end the program by closing the plot window_. **Do not send Ctrl-C until you have closed the plot window, or else the data will not be written to the file!** You should see `***************FILES WRITTEN**************` appear in the terminal once you have done this. You can then kill the remaining nodes with `Ctrl-C`. 


## Plotting and analyzing the data ##

To use `plot.py` and `analyze.py`, you must understand their flags:

+ `-h` will display usage information
+ `-n` will normalize the coefficients before plotting/analyzing
+ `-D` will take the first derivative of the curves
+ `-DD` will take the second derivative of the curves

Specific to `plot.py`:

+ `-a` will plot curves for all containers
+ `-d dir` will plot the curve for the container with data in `dir`

`analyze.py` requires a subcommand, which should be placed _after_ any flags:

+ `cluster` will display plots of the data points transformed by PCA, and print their classes to the terminal
+ `plot` will plot the Legendre coefficients of all curves


Both scripts should work fine with either `python` or `rosrun`.

#####Examples:#####
- `python plot.py -d 014`: plot the data points for container #014, along with the fitted polynomial curve
- `python plot.py -anD`: plot the normalized first derivative of curves for all containers
- `python analyze.py -nD cluser`: determine and plot clusters of data
- `python analyze.py -nDD plot`: plot the Legendre coefficients for the normalized second derivative curves of all containers



## Curve fitting and classification code ##

Curve fitting is done via the function `fit_curve` in `curve_fitting.py`. The curve fitting is done using `scipy.optimize.minimze`, constrained by a least squares fit and nonnegative first derivative.

Classification is done in several steps:
1. The initial data for a pour is collected only once fluid starts actually flowing out.
2. This data is collected for a total of 0.5 radians, then it is sent to the classifier.
3. The classifier service fits a curve to the data. The curve is transformed into Legendre coefficients (**Important:** these coefficients are scaled on the domain [start\_angle, start\_angle+ 1.0]. This domain _must_ also be applied to coefficients for all curves in order for the classification to make sense).
4. The service then takes the deriviatve of this curve and normalizes the coefficients.
5. The classifier compares the normalized coefficients to other data points using k_means.
6. Given the class of the container, the classifier determines the average of curves for that class, then scales this curve down to match the starting angle of the container.


##To Do##

- Configure the Linux machine to allow this code to run in Python 3 (the right packages need to be installed with PIP and then the shebang headers at the tops of the scripts should be changed to `#!\usr\bin\env python3`)
- Change `analyze.py` and `plot.py` to store and retrieve coefficients using files instead of calculating them again each time the scripts are run.
- Thoughourly test classifier with a variety of starting points to determine whether the scaling of the Legendre polynomials will be affected too greatly
- Determine whether Gauss-Newton or Sequential Least Squares Programming (implemented within `scipy.optimize.minimize`) will work best to extrapolate the rest of the curve after classification provides an initial guess
- Integrate classification ROS service and optimization code into a new ROS node that can be used to test accuracy