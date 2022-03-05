/****************************************************************************\
 * Copyright (C) 2016 Infineon Technologies
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#pragma once

#include <royale/Definitions.hpp>
#include <royale/String.hpp>

namespace royale
{
    /**
    * Severity of an IEvent.
    */
    enum class EventSeverity
    {
        /** Information only event.
        */
        ROYALE_INFO = 0,
        /** Potential issue detected (e.g. soft overtemperature limit reached).
        */
        ROYALE_WARNING = 1,
        /** Errors occurred during operation.
        * The operation (e.g. recording or stream capture) has failed and was stopped.
        */
        ROYALE_ERROR = 2,
        /** Severe error was detected.
        * The corresponding ICameraDevice is no longer in an usable state.
        */
        ROYALE_FATAL = 3
    };

    /**
    * Type of an IEvent.
    */
    enum class EventType
    {
        ROYALE_CAPTURE_STREAM,
        ROYALE_DEVICE_DISCONNECTED,
        ROYALE_OVER_TEMPERATURE,
        ROYALE_RAW_FRAME_STATS
    };

    /**
    * Interface for anything to be passed via IEventListener.
    */
    class IEvent
    {
    public:
        virtual ~IEvent() = default;

        virtual royale::EventSeverity severity() const = 0;
        virtual const royale::String describe() const = 0;
        virtual royale::EventType type() const = 0;
    };
}
