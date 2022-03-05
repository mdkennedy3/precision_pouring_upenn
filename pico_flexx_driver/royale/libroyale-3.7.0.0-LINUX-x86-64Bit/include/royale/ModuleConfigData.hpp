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

#include <config/ModuleConfig.hpp>

namespace royale
{
    namespace config
    {
        /**
        * The module configurations are not exported directly from the library.
        * The full list is available via royale::config::getUsbProbeDataRoyale()
        */
        namespace moduleconfig
        {
            /**
            * Configuration for the PicoS with Enclustra firmware.
            */
            extern const royale::config::ModuleConfig PicoS;

            /**
            * Configuration for the PicoFlexx with Enclustra firmware.
            */
            extern const royale::config::ModuleConfig PicoFlexxU6;

            /**
            * Configuration for the EvalBoard with Enclustra firmware.
            */
            extern const royale::config::ModuleConfig EvalBoard;

            /**
            * Configuration for the EvalBoard
            * "Infineon IRS16x5C Evaluation Kit LED"
            * with Enclustra firmware.
            */
            extern const royale::config::ModuleConfig EvalBoardIRS16x5CLED;

            /**
            * Configuration for the C2 camera module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            */
            extern const royale::config::ModuleConfig C2Uvc;

            /**
            * Configuration for the CX3 bring-up board with UVC firmware.
            *
            * This is a bring-up board, expected to be used by hardware developers to test new
            * modules, and as a template for the ModuleConfigs for those modules.
            */
            extern const royale::config::ModuleConfig CX3Uvc;

            /**
            * Configuration for the Skylla camera prototype module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            */
            extern const royale::config::ModuleConfig Skylla;

            /**
            * Configuration for the Charybdis camera module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            */
            extern const royale::config::ModuleConfig Charybdis;

            /**
            * Configuration for pico maxx (first diffuser)
            */
            extern const royale::config::ModuleConfig PicoMaxx1;

            /**
            * Configuration for pico maxx (second diffuser)
            */
            extern const royale::config::ModuleConfig PicoMaxx2;

            /**
            * Configuration for pico monstar (first diffuser)
            */
            extern const royale::config::ModuleConfig PicoMonstar1;

            /**
            * Configuration for pico monstar (second diffuser)
            */
            extern const royale::config::ModuleConfig PicoMonstar2;

            /**
            * Configuration for the Daedalus camera module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            */
            extern const royale::config::ModuleConfig Daedalus;
        }
    }
}
