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
            * Configuration for the PicoFlexx with Enclustra firmware, and additional calibration at
            * 15 megahertz.
            */
            extern const royale::config::ModuleConfig PicoFlexx15MHz;

            /**
            * Configuration for the PicoFlexx with Enclustra firmware, and 940nm illumination.
            */
            extern const royale::config::ModuleConfig PicoFlexx940nm;

            /**
            * Configuration for the EvalBoard with Enclustra firmware.
            */
            extern const royale::config::ModuleConfig EvalBoard;

            /**
            * Configuration for the C2 camera module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            *
            * This is also used as the fallback configuration for the Animator CX3 bring-up board
            * with both UVC and Amundsen firmware.  However, users of the bring-up board should
            * configure Royale to match their hardware.
            */
            extern const royale::config::ModuleConfig C2Uvc;

            /**
            * Configuration for the Skylla camera prototype module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            * Variant 1 :
            * K6 / 60x45 / 850nm / 96 / LE
            */
            extern const royale::config::ModuleConfig Skylla1;

            /**
            * Configuration for the Skylla camera prototype module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            * Variant 2 :
            * K6 / 60x45 / 850nm / 101 / PO
            */
            extern const royale::config::ModuleConfig Skylla2;

            /**
            * Configuration for the Skylla camera prototype module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            * Variant 3 :
            * K6 / 60x45 / 945nm / 101 / PO
            */
            extern const royale::config::ModuleConfig Skylla3;

            /**
            * Configuration for the Skylla camera prototype module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            * Default variant with only a calibration use case.
            */
            extern const royale::config::ModuleConfig SkyllaDefault;

            /**
            * Configuration for the Charybdis camera module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            */
            extern const royale::config::ModuleConfig Charybdis;

            /**
            * Configuration for the Charybdis camera module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            * Default variant with only one calibration use case.
            */
            extern const royale::config::ModuleConfig CharybdisDefault;

            /**
            * Configuration for pico maxx (first diffuser)
            */
            extern const royale::config::ModuleConfig PicoMaxx1;

            /**
            * Configuration for pico maxx (second diffuser)
            */
            extern const royale::config::ModuleConfig PicoMaxx2;

            /**
            * Configuration for pico maxx 850nm rev 4 with glass lens
            */
            extern const royale::config::ModuleConfig PicoMaxx850nmGlass;

            /**
            * Configuration for pico maxx 940nm
            */
            extern const royale::config::ModuleConfig PicoMaxx940nm;

            /**
            * Default configuration for pico maxx only one calibration
            * use case
            */
            extern const royale::config::ModuleConfig PicoMaxxDefault;

            /**
            * Configuration for pico monstar (first diffuser)
            */
            extern const royale::config::ModuleConfig PicoMonstar1;

            /**
            * Configuration for pico monstar (second diffuser)
            */
            extern const royale::config::ModuleConfig PicoMonstar2;

            /**
            * Configuration for pico monstar 850nm rev 4 with glass lens
            */
            extern const royale::config::ModuleConfig PicoMonstar850nmGlass;

            /**
            * Configuration for pico monstar 940nm
            */
            extern const royale::config::ModuleConfig PicoMonstar940nm;

            /**
            * Default configuration for pico monstar with only one calibration
            * use case
            */
            extern const royale::config::ModuleConfig PicoMonstarDefault;



			extern const royale::config::ModuleConfig G82100;
			extern const royale::config::ModuleConfig G82101;
            /**
            * Configuration for the Daedalus camera module connected via UVC.
            * Variant 1 :
            * K6 / 60x45 / 850nm / 202 / PO
            */
            extern const royale::config::ModuleConfig Daedalus1;

            /**
            * Configuration for the Daedalus camera module connected via UVC.
            * Variant 2 :
            * K7L / 60x45 / 850nm / 202 / PO
            */
            extern const royale::config::ModuleConfig Daedalus2;

            /**
            * Configuration for the Daedalus camera module connected via UVC.
            * Variant 3 :
            * K8 / 90x70 / 850nm / 202 / PO
            */
            extern const royale::config::ModuleConfig Daedalus3;

            /**
            * Configuration for the Daedalus camera module connected via UVC.
            * Variant 4 :
            * K6 / 60x45 / 945nm / 202 / PO
            */
            extern const royale::config::ModuleConfig Daedalus4;

            /**
            * Default configuration for a Daedalus camera module where no product
            * identifier was found or doesn't match the other variants. This only
            * offers a calibration use case.
            */
            extern const royale::config::ModuleConfig DaedalusDefault;

            /**
            * Configuration for the Alea camera modules with 850nm VCSEL.
            * Variant 1 + 2:
            * 60x45 / 850nm / 202 / PO
            */
            extern const royale::config::ModuleConfig Alea850nm;

            /**
            * Configuration for the Alea camera modules with 945nm VCSEL.
            * Variant 3 + 4:
            * 60x45 / 945nm / 202 / PO
            */
            extern const royale::config::ModuleConfig Alea945nm;

            /**
            * Configuration for the Apollo camera module connected via UVC.
            * (The device uses a CX3-based USB adaptor).
            */
            extern const royale::config::ModuleConfig Apollo;
        }
    }
}
