This is a fork of the PnP-Focus project started by Anat available here:

https://github.com/aruangra/PnP-Focus

Since there are significant Changes with this V2.0 version compared to the original project, let's start fresh with the features.

# PnP-Focus Version 2.0

## Objective:
To build a focus controller which takes arduino controller boards and plug-in controller shields to assemble a focus controller with minimal soldering or wiring.

## Two versions of the project:
* Standalone PNP-Focus Controller
* PnP-Focus Stick Controller




## Features:
* Manual control.
* LCD display (Optional)
* Temperature sensor.
* Local manual motor control using automatic increasing speed changes.
* Independent motor speeds for local button focus position and ASCOM commanded position
* Stepper motor output power down after a positional move to prevent motor from heating due to quiescent current.
* Ability to drive Uni-Polar or Bi-Polar stepper motors.
* Full step and half step motor movements.  
* Switching between full step and half step automatically converts the position counter to new step count and correct transition step code.
* Ability to save position and step mode to non-volatile memory and recall on power-up to restore unit to previously used configuration

* Compatible with Moonlite DRO ASCOM driver with the following compatibility features:
    * Full selection and operation of moonlight DRO ASCOM step speeds using 16, 32, 64, 125, 250 steps/second.
    * Temperature sensor for temperature compensation by ASCOM driver.  Temperature sensor moved to pin A2.
    * ASCOM connection locks mode to driver selected Full step or Half step  mode.
    * ASCOM stop or disconnect records stepper position and step mode to non-volatile memory.

* Additional features added when operating in ASCOM connection which are not provided by    Moonlite Technologies DRO driver:
    * Backlash compensation for outward movement to compensate for gravity and to ensure consistent approach to focus position in either direction.
    * ASCOM moves use soft start with speeds increasing at driver speed intervals, up to the selected driver step speed setting.
    * ASCOM move command have reducing speed as target position is approached to to ensure positional accuracy and repeatability.
    * Provides the ability to use high speed focus slews without loss of accuracy or repeatability.
* Compatible with commercially available focus drives using stepper motors
* Works with the popular stepper motors:
    * ABS3008-002  (12V) RoboFocus Stepper Motor
    * LSG35012F76P (12V) Moonlite Telescope Hi-Resolution Stepper Motor
    * 28BYJ-48  (5V and 12V)
    * 17HS13-0404S-PG27 (12V)


## Installation:

The Original Installation upload approach is obsolete and not use with this version.  Requires users to use the Arduino Integrated Development Interface (IDE) program to load the Arduino firmware files.  The use must modify the user configuration header area to match the code to their specific build and then compile and upload the file to the arduino.  All the individual files are available 
https://github.com/benkot/PnP-Focus 

The entire project files are availble in a zip download to make it easier to prepare and install the firmware.
https://github.com/benkot/PnP-Focus/blob/master/PnPFocus-V2.0.zip

## Documentation:
Project manual describing the necessary hardware, installation, assembly, user controls and operating principles is available at 
https://github.com/benkot/PnP-Focus/blob/master/PNP%20Focuser%20V2.0%20Documentation.pdf
