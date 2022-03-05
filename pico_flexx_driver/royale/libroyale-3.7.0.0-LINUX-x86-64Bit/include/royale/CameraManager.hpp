/****************************************************************************\
 * Copyright (C) 2015 Infineon Technologies
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#pragma once

#include <memory>

#include <royale/Definitions.hpp>
#include <royale/ICameraDevice.hpp>
#include <royale/Vector.hpp>
#include <royale/String.hpp>
#include <royale/TriggerMode.hpp>

namespace royale
{
    /**
     *  The CameraManager is responsible for detecting and creating instances of `ICameraDevices`
     *  one for each connected (supported) camera device. Depending on the provided activation code access `Level 2` or
     *  `Level 3` can be created. Due to eye safety reasons, `Level 3` is for internal purposes only.  Once a known
     *  time-of-flight device is detected, the according communication (e.g. via USB)
     *  is established and the camera device is ready.
     */
    class CameraManager
    {
    public:
        /**
         *  Constructor of the CameraManager. An empty activationCode only allows to get an ICameraDevice.
         *  A valid activation code also allows to gain `Level 2` or `Level 3 ` access rights.
         */
        ROYALE_API CameraManager (const royale::String &activationCode = "");

        /**
         *  Destructor of the CameraManager
         */
        ROYALE_API ~CameraManager();

        /**
        *  Retrieves the access level to the given activation code.
        */
        ROYALE_API static royale::CameraAccessLevel getAccessLevel (const royale::String &activationCode = "");

        /**
         *  Returns the list of connected camera modules identified by a unique ID (serial number).
         *  This call tries to connect to each plugged-in camera and queries for its unique serial number.
         *  Found cameras need to be fetched by calling createCamera(). This in turn moves the ownership
         *  of the CameraDevice to the caller of createCamera(). Calling getConnectedCameraList() twice
         *  will automatically close all unused ICameraDevices that were returned from the first call.
         *  <B>Calling this function twice is not the expected usage for this function!</B>
         *  Once the scope of CameraManager ends, all (other) unused ICameraDevices will also
         *  be closed automatically. The createCamera() keeps the ICameraDevice beyond the scope
         *  of the CameraManager since the ownership is given to the caller.
         *
         *  <B>WARNING: please also only have one instance of CameraManager at the same time! royale does not support
         *  multiple instances of CameraManager in parallel.</B>
         */
#if defined(TARGET_PLATFORM_ANDROID)
        /**
         * \param androidUsbDeviceFD file descriptor of the USB device
         * \param androidUsbDevicePath path to the USB device file
         */
#endif
        /**
         * \return list of connected camera IDs
         */

#if defined(TARGET_PLATFORM_ANDROID)
        ROYALE_API royale::Vector<royale::String> getConnectedCameraList (uint32_t androidUsbDeviceFD,
                uint32_t androidUsbDeviceVid,
                uint32_t androidUsbDevicePid);
#else
        ROYALE_API royale::Vector<royale::String> getConnectedCameraList();
#endif

        /**
         *  Creates a master or slave camera object ICameraDevice identified by its ID. The ID can be
         *
         *  - The ID which is returned by getConnectedCameraList (representing a physically connected camera)
         *  - A given filename of a previously recorded stream
         *
         *  If the ID or filename are not correct, a nullptr will be returned. The ownership is transfered
         *  to the caller, which means that the ICameraDevice is still valid once the CameraManager is out of scope.
         *
         *  In case of a given filename, the returned ICameraDevice can also be dynamically casted to an IReplay
         *  interface which offers more playback functionality.
         *
         *  If the camera is opened as a slave it will not receive a start signal from Royale, but will wait
         *  for the external trigger signal. Please have a look at the master/slave example which shows how to
         *  deal with multiple cameras.
         *
         *  \param cameraId Unique ID either the ID returned from getConnectedCameraList of a filename for a recorded stream
         *  \param mode Tell Royale to open this camera as master or slave.
         *
         *  \return ICameraDevice object if ID was found, nullptr otherwise
         */
        ROYALE_API std::unique_ptr<royale::ICameraDevice> createCamera (const royale::String &cameraId, const royale::TriggerMode mode = TriggerMode::MASTER);

        /**
         *  Returns key information for the camera module identified by its ID.
         *
         *  @param cameraId Unique ID of the camera which was returned from getConnectedCameraList()
         *
         *  @return CameraCharacteristics object with key information for the camera module
         */
        //CameraCharacteristics getCameraCharacteristics(const std::string &cameraId);

    private:
        struct CameraManagerData;
        std::unique_ptr<CameraManagerData> m_data;
    };
}
