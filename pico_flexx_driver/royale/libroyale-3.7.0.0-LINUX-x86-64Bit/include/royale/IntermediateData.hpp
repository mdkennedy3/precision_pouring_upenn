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

#include <royale/Definitions.hpp>
#include <royale/DepthData.hpp>
#include <royale/Vector.hpp>
#include <royale/StreamId.hpp>
#include <memory>
#include <cstdint>
#include <chrono>

namespace royale
{
    /**
     *  In addition to the standard depth point, the intermediate point also stores
     *  information which is calculated as temporaries in the processing pipeline.
     *  Distance : Radial distance for each point (in meter)
     *  Amplitude : Grayscale image that also provides a hint on the amount of reflected light.
     *              The values are positive, but the range depends on the camera that is used.
     *  Intensity : Intensity image (values can be negative in some cases)
     *  Flags : Flag image that shows invalid pixels. For a description of the flags please refer to the Spectre documentation
     */
    struct IntermediatePoint
    {
        float       distance;             //!< distance value of the current sequence
        float       amplitude;            //!< amplitude value of the current sequence
        float       intensity;            //!< intensity value of the current sequence
        uint32_t    flags;                //!< flags of the current sequence
    };

    /**
     *  This structure defines the Intermediate depth data which is delivered through the callback
     *  if the user has access level 2 for the CameraDevice.
     */
    struct IntermediateData
    {
        int                                                         version;                //!< version number of the data format
        std::chrono::microseconds                                   timeStamp;              //!< timestamp in microseconds precision (time since epoch 1970)
        StreamId                                                    streamId;               //!< stream which produced the data
        uint16_t                                                    width;                  //!< width of distance image
        uint16_t                                                    height;                 //!< height of distance image
        royale::Vector<royale::Vector<royale::IntermediatePoint>>   points;                 //!< array of intermediate points for each sequence
        royale::Vector<uint32_t>                                    modulationFrequencies;  //!< modulation frequencies for each sequence
        royale::Vector<uint32_t>                                    exposureTimes;          //!< integration times for each sequence
        uint32_t                                                    numFrequencies;         //!< number of processed frequencies
    };
}
