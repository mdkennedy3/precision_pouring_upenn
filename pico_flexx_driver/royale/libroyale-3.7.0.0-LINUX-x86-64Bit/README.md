Introduction
============

The **Royale** software package provides a light-weight camera framework for time-of-flight (ToF) cameras. While
being tailored to PMD cameras, the framework enables partners and customers to evaluate and/or
integrate 3D TOF technology on/in their target platform. This reduces time to first demo and time to
market.

Royale contains all the logic which is required to operate a ToF based camera. The user need not
care about setting registers, but can conveniently control the camera via a high-level interface.
The Royale framework is completely designed in C++ using the C++11 standard.

Royale officially supports the **CamBoard pico flexx**, **CamBoard pico maxx** and **CamBoard pico monstar** cameras.

Operating Systems
-----------------

Royale supports the following operating systems:

- Windows 7/8
- Linux (tested on Ubuntu 14.04)
- OS X (tested on El Capitan 10.11)
- Android (minimum android version 4.1.0. Tested on Android 4.4.2 and 5.1.1)

Hardware Requirements
---------------------

Royale is tested on the following hardware configurations:

- PC, Intel i5-660, 3.3 GHz, 2 cores (64 bit)
- PC, Intel i7-2600, 3.4 GHz, 4 cores (64 bit)
- PC, Intel i7-3770, 3.4 GHz, 4 cores (64 bit)
- MacBook Pro Late 2013, Intel Core i5, 2.4 Ghz (64 bit)
- Samsung Galaxy S6
- Samsung Galaxy S7
- Samsung Galaxy Note 10.1(SM-P605)
- Samsung Galaxy Note Pro 12.2’’ Wifi
- LGE G Pad 8.3’’
- OnePlus 2
- Nexus 6
- Huawei Mate 9


Getting Started
===============

Plug in your supported PMD camera to a free USB port of your target device. It is suggested to use a free USB 3 plug.
For some Android devices you may need a powered USB hub for proper usage. For Windows-based
systems, please install the low-level camera driver which is part of the delivery package.
For Linux-based systems, make sure that you have proper permissions to the USB device. The installation
package contains a proper rules file which can be used. For more details, please see the
drivers/udev/README file (for Linux distributions).

Once the camera is attached to a free USB port, and the drivers are in place, you may start the
**royaleviewer** application which gives you a first indication, if the camera is working on your target
system.  The royaleviewer displays a 2D or a 3D representation of the captured depth data. By clicking
on the *Start* button, the capture mode is started and you should see a colored depth map.

Use cases
---------

For 3D sensing a set of key applications and corresponding use cases have been determined and a corresponding
set of processing parameters and camera options have been derived. Those values can only be changed on the 2nd and 3rd level of the `ICameraDevice` interface.

Depending on the module you're using the use cases that are offered by Royale may vary.

A **use case** is a pre-defined set of parameters reflecting a certain scenario.
For applications doing hand tracking, for example, it may be beneficial to acquire frames at a high frame rate, whereas the
range isn't as important. Scanning applications on the other hand will benefit from the range and stability of data that is acquired
with less frames per second, but more overall phases and higher exposure times.

The use case encodes three different type of information:

- number of raw frames being captured for generating the depth image
- number of frames per seconds
- the maximal exposure time for a raw frame capture event

A *raw frame* contains a 12-bit image which is returned from the ToF imager using a single
phase measurement. The user typically does not need to set the internals of the ToF imager
since Royale is providing this low-level information.

Please take a look at the document describing your module for more information on which use cases are offered.

Access Levels
-------------

Royale offers four different access levels, each one representing a specific
user base.

> For eye-safety reasons access for devices labelled as `Laser Class 1` is only permitted on `Level 1` and `Level 2`.

- **Level 1**
: is for operating the camera in a default configured way. Only a limited set of functions
  are exposed which change the behaviour of the camera module. This level is typically
  used by application developers who want to integrate Royale into their applications
  serving high quality depth data.

- **Level 2**
: is for users who also need to handle raw data in addition to depth data. This level also
  allows alteration of internal processing parameters in order to evaluate the depth data
  generation and optimise for certain use cases. This level is typically used by users who
  want to evaluate and optimise depth but also raw data.

- **Level 3 and Level 4**
: these levels are for developing new camera modules, and also for optimising the internal
  parameters of the camera module. This mode must be run with caution since it may not
  only damage the hardware but also harm your health (laser class 1 eye safety is not
  guaranteed, particularly with prototype hardware). This level is restricted to internal
  employees only and will not be exposed to customers.

#### Level Activation

The different user level can be activated via the `ICameraDevice` Interface by
entering the correct access codes on construction via the `CameraDeviceManager`.
`Level 1` is activated by default, no code is required. `Level 2`, `Level 3` and `Level 4` can
only be activated with a special activation code (three different codes, one for each level).
The higher level also inherits all functionality of the lower levels.

Mixed Mode
----------

The mixed mode can be used to run two or more different modes at the same time, by capturing frames that fit into separate use cases.
It can be selected like any other use case. If you select a mixed mode use case you have to provide the stream ID you want to change to 
some of the Royale API calls :

~~~
royale::CameraStatus setExposureTime (uint32_t exposureTime, 
                                      royale::StreamId streamId = 0)
royale::CameraStatus setExposureMode (royale::ExposureMode exposureMode, 
                                      royale::StreamId streamId = 0)
royale::CameraStatus getExposureMode (royale::ExposureMode &exposureMode, 
                                      royale::StreamId streamId = 0)
royale::CameraStatus getExposureLimits (royale::Pair<uint32_t, uint32_t> &exposureLimits, 
                                        royale::StreamId streamId = 0)
~~~

For Level 2 :

~~~
royale::CameraStatus setProcessingParameters (
                                   const royale::ProcessingParameterVector &parameters, 
                                   uint16_t streamId = 0)
royale::CameraStatus getProcessingParameters (
                                   royale::ProcessingParameterVector &parameters, 
                                   uint16_t streamId = 0)
~~~

If you don't provide the stream ID for these functions Royale will return an error. You can query the stream IDs of the current use case 
by calling 

~~~
royale::CameraStatus getStreams (royale::Vector<royale::StreamId> &streams)
~~~

Also the functions dealing with frame rates are currently not supported, but will be implemented in the future.
As with normal use cases the data is provided in a callback function. The stream ID of the current data can be found in the 
data object you receive.
  
SDK and Examples
================

Besides the royaleviewer application, the package also provides a Royale SDK which can be used
to develop applications using the PMD camera.

There are two samples included in the delivery package which should give you a brief overview
about the exposed API. The *doc* directory offers a detailed description of the Royale API which
is a good starting point as well.

The Royale framework currently offers capturing in free running mode only. During capturing the exposure
time can be altered up to the maximal specified values defined by the use case.

The data is provided using a callback based interface. The consumer must register a listener to
the camera device and retrieves the depth data. The high-level API consists of the following interfaces:

- **ICameraDevice**
  : Main interface which represents the physical camera. The user will mostly work with this interface
    to talk to the physical camera device.

- **CameraManager**
  : The main component which can query for connected/supported cameras and is
    able to generate the proper ICameraDevice.

- **CameraStatus**
  : Gives information about the internal state of the system. Most methods will return a proper
    error code to the caller.

- **IDepthDataListener**
  : Interface which needs to be implemented in order to retrieve the depth data.

- **DepthData**
  : Contains the processed depth data which can be consumed by the user.
    Furthermore it contains the exposureTimes for all frames within one DepthData


Compilation
-----------

The package contains all necessary headers and libraries including two samples which should
provide a first get-in-touch. The samples also contain a CMakeLists.txt file which allows
to compile the code on different platforms.

Please note that you need a C++11 compliant compiler:

- Windows: Visual Studio 2013 or higher
- Linux: gcc 4.8 or higher
- OS X: gcc 4.8 or higher or clang 6.1 or higher

You will also need :

- CMake: 3.1 or higher

### Linux

Open up your favorite console:

~~~
cd samples/sampleCameraInfo
mkdir build
cd build
cmake ..
make
~~~

### Windows

Open up your development shell (e.g. Visual Studio command line):

~~~
cd samples/sampleCameraInfo
mkdir build
cd build
cmake -G "Visual Studio 12 2013" ..
~~~

Depending on the IDE you're using you have to adapt the generator used by
CMake! Please note that the generators for 64Bit Visual Studio builds differ
from the ones for 32Bit!
Then open up your generated solution file with your Visual Studio IDE.

### OS X

Open up your favorite console:

~~~
cd samples/sampleCameraInfo
mkdir build
cd build
cmake -G Xcode ..
~~~

Then open up your generated project file with Xcode IDE.

The package also provides a cmake config file for properly building Royale applications.
The cmake file can be found under the share directory.

###Requirements

The package requires to have the libusb available under Linux-based and macOS systems.

Examples
--------

The following examples demonstrate the current API and its usage.

###Create a CameraDevice

The physical depth camera module is represented by the `ICameraDevice`. This object is automatically
generated by the `CameraManager`. During initialization, the correct configuration parameters for
the camera module are loaded and the module/imager is initialized. Here is an example for how to
instantiate a `ICameraDevice`:

~~~
std::unique_ptr<ICameraDevice> camera;
CameraManager manager;

auto connectedCameras = manager.getConnectedCameraList();

// get the first found camera, assuming one camera was found
camera = manager.createCamera(connectedCameras[0]);
~~~

###Set a Use Case

The camera can be operated in pre-defined use cases. The following code snippet shows, how to retrieve the available
use cases and select the first one:

~~~
// error checking omitted for clarity
royale::Vector<royale::String> useCases;
royale::CameraStatus ret = camera->getUseCases(useCases);
ret = camera->setUseCase(useCases[0]);
~~~

Since the use case provides a maximal exposure time, the time can be set during runtime by
using the setExposureTime() method. The pre-defined use cases make sure that eye safety is
guaranteed for cameras using laser illumination.

~~~
royale::CameraStatus ret = camera->setExposureTime(600);
~~~

###Start Capture Mode

After setting up the capture mode, the camera can start capturing. Therefore,
a listener needs to be registered:

~~~
royale::CameraStatus ret = camera->registerDataListener(listener);
~~~

The listener implementation needs to be derived from the following interface:

~~~
class IDepthDataListener
{
public:
    virtual ~IDepthDataListener() {}
    virtual void onNewData (const DepthData * data) = 0;
};
~~~

After registering a listener, the camera capture mode can be started:

~~~
royale::CameraStatus ret = camera->startCapture();
~~~

The camera capture mode can be stopped again by the following code:

~~~
royale::CameraStatus ret = camera->stopCapture();
~~~

###Data Structure

#### Point Cloud Data

The Royale framework delivers 3D point cloud data with the following information for each voxel:

- X/Y/Z coordinates in object space [m]
- Grayscale information
- Depth noise
- Depth confidence

The point cloud data is a dense map of those values in order to maintain neighbourhood information.
For the point cloud Royale uses a right handed coordinate system (x -> right, y -> down, z -> in
viewing direction).
The following data structure is proposed for the 3D point cloud data:

~~~
struct DepthPoint
{
    float x;                 //!< X coordinate [meters]
    float y;                 //!< Y coordinate [meters]
    float z;                 //!< Z coordinate [meters]
    float noise;             //!< noise value [meters]
    uint16_t grayValue;      //!< 16-bit gray value
    uint8_t depthConfidence; //!< value 0 = bad, 255 = good
};

struct DepthData
{
    int                          version;         //!< version number of the data format
    std::chrono::microseconds    timeStamp;       //!< timestamp in microseconds precision
                                                  //   (time since epoch 1970)
    uint16_t                     width;           //!< width of depth image
    uint16_t                     height;          //!< height of depth image
    royale::Vector<uint32_t>     exposureTimes;   //!< exposureTimes retrieved from
                                                  //   CapturedUseCase
    royale::Vector<DepthPoint>   points;          //!< array of points
};
~~~

###Debugging
#### Microsoft Visual Studio

To help debugging royale::Vector, royale::Pair and royale::String we provide a Natvis file
for Visual Studio. Please take a look at the natvis.md file in the doc/natvis folder of your
installation.

Matlab
=========
In the delivery package for Windows you will find a Matlab wrapper for the Royale library.
After the installation it can be found in the matlab subfolder of your installation directory.
To use the wrapper you have to include this folder into your Matlab search paths.
We also included some examples to show the usage of the wrapper. They can also be found in the
matlab folder of your installation.

Reference
=========

FAQ: http://pmdtec.com/picofamily/

License
=========
See royale_license.txt.

Parts of the software covered by this License Agreement (royale_license.txt) are using libusb under LGPL 2.1, QT5.5 under
LGPL 3.0, gradle wrapper under the Apache 2.0 license, libuvc under a BSD-style license and CyAPI under the Cypress Software License Agreement
(cypress_license.txt). Spectre is using Kiss FFT licensed under 
a BSD-style license. The documentation is created using Doxygen, which uses jquery and sizzle, both are licensed under the MIT license. 
The text of the GPL 3.0 is also provided because the LGPL 3.0, 
although more permissive, is written as a supplement to the GPL 3.0 and requires both texts.

