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
 * Drives a bipolar with 4 wires Stepper motor.
 *
 * When wiring multiple ArdumotoStepper motors to a microcontroller, you quickly run
 * out of output pins, with each motor requiring 4 connections.
 *
 * By making use of the fact that at any time two of the four motor coils are
 * the inverse of the other two, the number of control connections can be
 * reduced from 4 to 2 for the unipolar and bipolar motors.
 *
 * The sequence of control signals for 4 control wires is as follows:
 *
 * Step C0 C1 C2 C3
 *    1  1  0  1  0
 *    2  0  1  1  0
 *    3  0  1  0  1
 *    4  1  0  0  1
 *
 *
 * The sequence of halfstep control signals for 4 control wires is as follows:
 *  Step C0 C1 C2 C3
 *     1  1  0  1  0
 *     2  0  0  1  0
 *     3  0  1  1  0
 *     4  0  1  0  0
 *     5  0  1  0  1
 *     6  0  0  0  1
 *     7  1  0  0  1
 *     8  1  0  0  0
 *
 *
 *
 * The sequence of controls signals for 2 control wires is as follows
 * (columns C1 and C2 from above):
 *
 * Step C0 C1
 *    1  0  1
 *    2  1  1
 *    3  1  0
 *    4  0  0
 *
 * The circuits can be found at
 *
 * http://www.arduino.cc/en/Tutorial/ArdumotoStepper
 */

#include "Arduino.h"
#include "ArdumotoStepper.h"

 /*
  *   constructor for four-pin version
  *   Sets which wires should control the motor.
  */
ArdumotoStepper::ArdumotoStepper(int number_of_steps)
{
    this->step_number = 0;    // which step the motor is on
    this->direction = 0;      // motor direction
    this->last_step_time = 0; // time stamp in us of the last step taken
    this->number_of_steps = number_of_steps; // total number of steps for this motor

    // Arduino pins for the motor control connection:
    this->motor_pin_1 = 3;
    this->motor_pin_2 = 11;
    this->motor_dir_1 = 12;
    this->motor_dir_2 = 13;

    this->stepmode = FULLSTEP;  //  Always initialize in fullstep mode  so that number of steps is based on full steps and does not change

    // setup the pins on the microcontroller:
    pinMode(this->motor_pin_1, OUTPUT);
    pinMode(this->motor_pin_2, OUTPUT);
    pinMode(this->motor_dir_1, OUTPUT);
    pinMode(this->motor_dir_2, OUTPUT);
}

/*
 * Sets the speed in revs per minute
 *   input whatspeed = motor rpm
 *   output step_delay = usec/step
 */
void ArdumotoStepper::setSpeed(float whatSpeed)
{
    this->step_delay = 60L * 1000L * 1000L / (this->number_of_steps * whatSpeed);
    //While previous was correct this approach makes more logical sense  
}

/*  
*   report motor Step speed based on current step delay
*   output motorStepSpeed = steps/second
*/
short ArdumotoStepper::getStepSpeed()
{ //return speed in steps/second
  short motorStepSpeed;
  motorStepSpeed = 1000000/this->step_delay;
  return (motorStepSpeed);
}

/*
 * Sets the stepping mode
 */
void ArdumotoStepper::setStepMode(unsigned short mode)
{
  if (stepmode != mode)  // do not change if active mode is already same as commanded mode
  {
     this->stepmode = mode;

    if (mode == FULLSTEP)
    {
        this->number_of_steps = number_of_steps/2;    ///2
        step_number = step_number/2;   // need to get sequence close to equivalent step if mode changes
    }
    else
    {
        this->number_of_steps = number_of_steps*2;    //number of steps per full  doubles for half steps *2
        step_number = step_number*2;          // need to get sequence close to equivalent step if mode changes     
    }
  }
}

/*
 * Gets the stepping mode
 */
int ArdumotoStepper::getStepMode(void)
{
    return this->stepmode;
}

/*
 * release stepper (turn off power)
 */
void ArdumotoStepper::release(void)
{
    analogWrite(motor_pin_1, 0);
    analogWrite(motor_pin_2, 0);
}

/*
 * Moves the motor steps_to_move steps.  If the number is negative,
 * the motor moves in the reverse direction.
 */
void ArdumotoStepper::step(int steps_to_move)
{
    int steps_left = abs(steps_to_move);  // how many steps to take

    // determine direction based on whether steps_to_mode is + or -:
    if (steps_to_move > 0) { this->direction = 1; }
    if (steps_to_move < 0) { this->direction = 0; }

    // decrement the number of steps, moving one step each time:
    while (steps_left > 0)
    {
        unsigned long now = micros();
        // move only if the appropriate delay has passed:
        if (now - this->last_step_time >= this->step_delay)
        {
            // get the timeStamp of when you stepped:
            this->last_step_time = now;
            // increment or decrement the step number,
            // depending on direction:
            if (this->direction == 1)
            {
                this->step_number++;
                if (this->step_number == this->number_of_steps) {
                    this->step_number = 0;
                }
            }
            else
            {
                if (this->step_number == 0) {
                    this->step_number = this->number_of_steps;
                }
                this->step_number--;
            }
            // decrement the steps left:
            steps_left--;
            if (this->stepmode == FULLSTEP)
                stepMotor(this->step_number % 4);
            else
               halfstepMotor(this->step_number % 8);
        }
    }
}


/*
 * Moves the motor forward or backwards.
 */
void ArdumotoStepper::stepMotor(int thisStep)
{
  switch (thisStep) {
    case 0:  // 1010
      analogWrite(motor_pin_1, 255);
      digitalWrite(motor_dir_1, LOW);
      analogWrite(motor_pin_2, 255);
      digitalWrite(motor_dir_2, LOW);
    break;
    case 1:  // 0110
      analogWrite(motor_pin_1, 255);
      digitalWrite(motor_dir_1, HIGH);
      analogWrite(motor_pin_2, 255);
      digitalWrite(motor_dir_2, LOW);
    break;
    case 2:  //0101
      analogWrite(motor_pin_1, 255);
      digitalWrite(motor_dir_1, HIGH);
      analogWrite(motor_pin_2, 255);
      digitalWrite(motor_dir_2, HIGH);
    break;
    case 3:  //1001
      analogWrite(motor_pin_1, 255);
      digitalWrite(motor_dir_1, LOW);
      analogWrite(motor_pin_2, 255);
      digitalWrite(motor_dir_2, HIGH);
    break;
  }
}


void ArdumotoStepper::halfstepMotor(int thisStep)
{
  switch (thisStep) {
    case 0:    // 1010
      analogWrite(motor_pin_1, 255);
      digitalWrite(motor_dir_1, LOW);
      analogWrite(motor_pin_2, 255);
      digitalWrite(motor_dir_2, LOW);
    break;
    case 1:    // 0010
      analogWrite(motor_pin_1, 0);
      digitalWrite(motor_dir_1, LOW);
      analogWrite(motor_pin_2, 255);
      digitalWrite(motor_dir_2, LOW);
    break;
    case 2:    // 0110
      analogWrite(motor_pin_1, 255);
      digitalWrite(motor_dir_1, HIGH);
      analogWrite(motor_pin_2, 255);
      digitalWrite(motor_dir_2, LOW);
    break;
    case 3:    // 0100
      analogWrite(motor_pin_1, 255);
      digitalWrite(motor_dir_1, LOW);
      analogWrite(motor_pin_2, 0);
      digitalWrite(motor_dir_2, LOW);
    break;
    case 4:    //0101
      analogWrite(motor_pin_1, 255);
      digitalWrite(motor_dir_1, HIGH);
      analogWrite(motor_pin_2, 255);
      digitalWrite(motor_dir_2, HIGH);
    break;
    case 5:    //0001
      analogWrite(motor_pin_1, 0);
      digitalWrite(motor_dir_1, LOW);
      analogWrite(motor_pin_2, 255);
      digitalWrite(motor_dir_2, HIGH);
    break;
    case 6:    //1001
      analogWrite(motor_pin_1, 255);
      digitalWrite(motor_dir_1, LOW);
      analogWrite(motor_pin_2, 255);
      digitalWrite(motor_dir_2, HIGH);
    break;
    case 7:    //1000
      analogWrite(motor_pin_1, 255);
      digitalWrite(motor_dir_1, LOW);
      analogWrite(motor_pin_2, 0);
      digitalWrite(motor_dir_2, LOW);
    break;
  } 
}

/*
  version() returns the version of the library:
*/
int ArdumotoStepper::version(void)
{
    return 1;
}
