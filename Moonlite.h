/*
Moonlite.h - - Parser for the Moonlite serial focuser protocol - Version 1.0

History:
Version 1.0 - Author Jean-Philippe Bonnet
   First release

This file is part of the Moonlite library.

Moonlite library is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Moonlite library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Moonlite library.  If not, see <http://www.gnu.org/licenses/>.

 */

#ifndef Moonlite_h
#define Moonlite_h

#if ARDUINO < 100
#include <Wprogram.h>
#else
#include <Arduino.h>
#endif

//Definition of the Moonlite commands
#define ML_UNKNOWN_COMMAND 100;
#define ML_C 101    // Temperature convertion
#define ML_FG 110  // Go to target position
#define ML_FQ 111  // Stop movement
#define ML_GC 120  // Return temperature coefficient
#define ML_GD 121  // Return stepping delay (See command SD)
#define ML_GH 122  // Return 0xFF if the stepping mode is set to half-step otherwise returns 0x00
#define ML_GI 123  // Return "00" if the motor is not moving. "01" otherwisw 
#define ML_GN 124  // Return the target position set by the command SN
#define ML_GP 125  // Return the current position
#define ML_GT 126  // Return the current temperature
#define ML_GV 127  // Get the version of the firmware
#define ML_SC 130  // Set the new temperature coefficient
#define ML_SD 131  // Set the stepping delay (possible values are 02, 04, 08, 10 and 20 for a delay of respectively 250, 125, 63, 32, 16 step per second)
#define ML_SF 132  // Set full-step mode
#define ML_SH 133  // Set half-step mode
#define ML_SN 134  // Set the target position
#define ML_SP 135  // Set the new current position
#define ML_PLUS 140 // Activate the temperature compensation focusing
#define ML_MINUS 141 // Desactivate the temperature compensation focusing
#define ML_PO 150  // Set the temperature calibration offset
#define ML_GB 160 // Get the baklight LED value
#define ML_2UNKNOWN_COMMAND 200;
#define ML_2C 201    // Temperature convertion
#define ML_2FG 210  // Go to target position
#define ML_2FQ 211  // Stop movement
#define ML_2GC 220  // Return temperature coefficient
#define ML_2GD 221  // Return stepping delay (See command SD)
#define ML_2GH 222  // Return 0xFF if the stepping mode is set to half-step otherwise returns 0x00
#define ML_2GI 223  // Return "00" if the motor is not moving. "01" otherwisw 
#define ML_2GN 224  // Return the target position set by the command SN
#define ML_2GP 225  // Return the current position
#define ML_2GT 226  // Return the current temperature
#define ML_2GV 227  // Get the version of the firmware
#define ML_2SC 230  // Set the new temperature coefficient
#define ML_2SD 231  // Set the stepping delay (possible values are 02, 04, 08, 10 and 20 for a delay of respectively 250, 125, 63, 32, 16 step per second)
#define ML_2SF 232  // Set full-step mode
#define ML_2SH 233  // Set half-step mode
#define ML_2SN 234  // Set the target position
#define ML_2SP 235  // Set the new current position
#define ML_2PLUS 240 // Activate the temperature compensation focusing
#define ML_2MINUS 241 // Desactivate the temperature compensation focusing
#define ML_2PO 250  // Set the temperature calibration offset
#define ML_2GB 260 // Get the baklight LED value
#define ML_INPUT_BUFFER_SIZE 8 // Buffer size for the incomming command.
#define ML_OUTPUT_BUFFER_SIZE 5 // Buffer size for the answer message.

typedef struct MoonliteCommand_s
 {
   int commandID;
   long parameter;
} MoonliteCommand_t;

class Moonlite
{
 public:
  // Constructors:
  Moonlite();

  // Getters:
  MoonliteCommand_t getCommand();

  // Setters:
  void setAnswer(int nbChar, long answer);

  // Other public members
  void init(int baudRate);
  int isNewCommandAvailable();
  int Manage();

 private:
  MoonliteCommand_t currentCommand;
  int newCommandIsAvailable;
  char currentAsciiCommand[ML_INPUT_BUFFER_SIZE];
  char AsciiAnswer[ML_OUTPUT_BUFFER_SIZE];
  int currentAsciiIndex;
  void decodeCommand();
  static const int HexTable[16];

  void readNewAscii();
  long convert4CharToLong(char c1, char c2, char c3, char c4);
  long convert2CharToLong(char c1, char c2);
  long convert2CharToSignedLong(char c1, char c2);

  void convertLongToChar(long value, int nbChar, char *buffer);
};

#endif //Moonlite_h
