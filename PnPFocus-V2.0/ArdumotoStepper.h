/*
 * ArdumotoStepper.cpp - ArdumotoStepper library for Wiring/Arduino - Version 1.0
 *
 * Based on Arduino Stepper library. Original Contributors:
 * Original library        (0.1)   by Tom Igoe.
 * Two-wire modifications  (0.2)   by Sebastian Gassner
 * Combination version     (0.3)   by Tom Igoe and David Mellis
 * Bug fix for four-wire   (0.4)   by Tom Igoe, bug fix from Noah Shibley
 * High-speed stepping mod         by Eugene Kozlenko
 * Timer rollover fix              by Eugene Kozlenko
 * Five phase five wire    (1.1.0) by Ryan Orendorff
 *
 * ArdumotoStepper adaptation (1.0) by Alayn Gortazar (Barnetik Koop. Elk. Txikia)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *
 * Drives a 4 wired bipolar Stepper motor over SparkFun's Ardumoto shield.
 *
 * When wiring multiple ArdumotoStepper motors to a microcontroller, you quickly run
 * out of output pins, with each motor requiring 4 connections.
 *
 * By making use of the fact that at any time two of the four motor coils are
 * the inverse of the other two, the number of control connections can be
 * reduced from 4 to 2 for the unipolar and bipolar motors.
 *
 * A slightly modified circuit around a Darlington transistor array or an
 * L293 H-bridge connects to only 2 microcontroler pins, inverts the signals
 * received, and delivers the 4 (2 plus 2 inverted ones) output signals
 * required for driving a ArdumotoStepper motor. Similarly the Arduino motor shields
 * 2 direction pins may be used.
 *
 * The sequence of control signals for 5 phase, 5 control wires is as follows:
 *
 * Step C0 C1 C2 C3 C4
 *    1  0  1  1  0  1
 *    2  0  1  0  0  1
 *    3  0  1  0  1  1
 *    4  0  1  0  1  0
 *    5  1  1  0  1  0
 *    6  1  0  0  1  0
 *    7  1  0  1  1  0
 *    8  1  0  1  0  0
 *    9  1  0  1  0  1
 *   10  0  0  1  0  1
 *
 * The sequence of control signals for 4 control wires is as follows:
 *
 * Step C0 C1 C2 C3
 *    1  1  0  1  0
 *    2  0  1  1  0
 *    3  0  1  0  1
 *    4  1  0  0  1
 *
 * The circuits can be found at
 *
 * http://www.arduino.cc/en/Tutorial/ArdumotoStepper
 */

 // ensure this library description is only included once
#ifndef ArdumotoStepper_h
#define ArdumotoStepper_h


#define FULLSTEP 1
#define HALFSTEP 2


// library interface description
class ArdumotoStepper {
public:
    // constructors:
    ArdumotoStepper();

    // speed setter method:
    void setSpeed(unsigned short whatSpeed);

    // report Step Speed
    short getStepSpeed();

    // set step mode to FULLSTEP or HALFSTEP
    void setStepMode(unsigned short mode);

    // Get the stepping mode 
    int getStepMode(void);

    // release (turn off) stepper
    void release(void);

    // mover method:
    void step(int number_of_steps);

    int version(void);

private:
    void stepMotor(int this_step);
    void halfstepMotor(int this_step);

    int direction;            // Direction of rotation
    unsigned long step_delay; // delay between steps, in ms, based on speed
    int number_of_steps;      // number of steps this motor can take before repeat cycle
    int step_number;          // which step the motor is on
    int stepmode;             // FULLSTEP or HALFSTEP

    // motor pin numbers:
    int motor_pin_1;
    int motor_pin_2;
    int motor_dir_1;
    int motor_dir_2;

    unsigned long last_step_time; // time stamp in us of when the last step was taken
};

#endif
