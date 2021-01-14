// Moonlite-compatible stepper controller
//
// Original code by orly.andico@gmail.com, 13 April 2014
// Modified by Anat Ruangrassamee (aruangra@yahoo.com), 26 September 2017
// Modified V2.0 by Todd Benko (Dtdastro@gmail.com), 01 November, 2020

//*****************************************************************************************************************************************************************
//  User configurations
//  Set to identify what options are installed in the build you will program
//  
//  Ardumoto board can be purchased with different pin assignments 
//  The standard for this programs uses
//  D3 -A motor PWM drive 
//  D11-B motor PWM Drive
//  D12-A Dir
//  D13-B Dir
//  If your Ardumoto board uses different pin assignment, you must change assignment in to match in ArdumotoStepper.cpp
//  program is default setup to work with 
//  ardumoto shield which uses pins 3,11,12,13
//  nano motor shield which uses pins 3,11,12,13
//  
//  Many of the ardumoto boards do not have the logic voltage selection applied.  The Leonardo uses 5V logic.
//  A solder connection must be made to the 5V pad on the logic-V selection pad on the Ardumoto board.  
//  Nano Motor shield already has 5V logic selected.  The ardumoto library only provides full step operations 
//  because the dir pin is what swaps polarity of both coils on the stepper wire and there is no individual control of each stepper coil
//  
//  Program tested to work with both the 
//  Leonardo AVR board
//  MICRO AVR board
//  
//  Program can be used with various Arduino boards provided you select the correct board in Tools-Board menu and
//  which serial port connected in Tools-Port selection
//  
//  The following are usable definitions which can be changed to match your specific devices and assembly configuration
////////////////////////////////////////////////////////////////////////////////

//Require a definition stepsPerRevolution which matches your stepper motor.  
const int stepsPerRevolution = 3600;    // change this to fit the number of steps per revolution for your motor
//  Moonlite focus stepper LSG35012F76P = 3600 full steps/revolution 
//  Nema17 with gearhead 27:1 number 17HS13-0404S-PG27 = 5373 full steps/revolution
// 64:1 geared 28BYJ-48 = 2038 Full steps / revolution


// LCD1602Shield settings
// if the LCD1602 display shield is installed following line should be #define LCD1602Shield , if not installed it should be //#define LCD1602Shield 
// this is literally the only difference between leonardo and micro use.  LCD1602 shield remains optionional for Leonardo standalone build. 
//#define LCD1602Shield   

// LCD1602 starting brightness at powerup
const short userStartBrightness = 100;                  // Change value to personal choice for startup brightnes 0=off 10= dim 200= bright  value range is 0 to 255

// Backlash Compensation: Many geared focus motors have backlash in the gears when changing direction.  The motor shaft must turn x number of steps before the output shaft will turn.
// The way around the backlash problem is to always approach the target focus position from the same direction.  Backlash compensation is applied to focus movements that extend the 
// focuser position. The backlash compensation only apply to ASCOM driver target position changes.   When the focuser is racked outward the focus position will overshoot the 
// target position  by the backlashValue amount and then be racked back in to the target position.  This approach provide constant approach to the focus position and 
// removes the backlash in the gears and removes gravitational effects on position change.  

const unsigned short backlashValue=20;                 // increase this value to have a positive overshoot and rack back in to compensate for focuser backlash. 
                                                       // zero implies no backlash, value of 10 will rack out  10 additonal steps and the 10 steps 
                                                       // inward to finish at the target position.
// Temperature Units:
const bool displayFahrenheit = false;                  // false =  temperatures in Celsius., true = temperatures in Fahrenheit.

// Enable Half step mode
const bool enableHalfStepMode =  true;                // If stepper motor stalls/studders in half step mode, can disable half step mode so only full step is selected.
                                                      // true = half and full step modes available, false = only full step mode is enabled.

///////////////////////////////////////////////////////// 
//   End of User configuration
//   Do not change anything beyond this point 
//   unless you wish to embark on Arduino programming 
//******************************************************************************************************************************************************************************

// Programming note:  Following are code block examples which are used to apply code to be compiled if LCD shield is are installed
#ifdef LCD1602Shield  
  // code which will be compiled for use when LCD1602 shield is installed
#endif  // LCD1602Shield

////////////////////////////////////////////////////////////
// Common Libraries includes and intializations declarations
////////////////////////////////////////////////////////////
#include <EEPROM.h>
#include "Moonlite.h"                             // use serial commands to communicate with moonlite driver
#include "ArdumotoStepper.h"                      // use the ardumoto shield or nano motor shield
#include "OneWire.h"                              // use one wire protocol for dallas temperature sensor
#include "DallasTemperature.h"                    // use dallas temperature sensors DS18B20, or DS18B22, or DS18B25
#include <LiquidCrystal.h>                        // use the 1602 LCD display shield

// Definitions


///////////////////////////////////////////////////////////
// Temperature Sensor connection
///////////////////////////////////////////////////////////
#define           ONE_WIRE_BUS A2                 // Data wire is plugged into pin A2 on the Arduino  Needed to move temperature to A2 because Nano shield used A0 and A1.
OneWire           oneWire(ONE_WIRE_BUS);          // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);              // Pass oneWire reference to Dallas Temperature.

///////////////////////////
// Temperature Signals
///////////////////////////
float             tempC = 0;                      // temperature C reading from DS18B20
int               tempInitiated = 0;              // Temperature request initiated

///////////////////////////
// Ascom Serial Protocol
///////////////////////////
Moonlite SerialProtocol;

///////////////////////////
// Stepper
///////////////////////////

ArdumotoStepper myStepper(stepsPerRevolution);    //declare Ardumoto call

///////////////////////////
// Motor Control Signals
///////////////////////////
int               isRunning = 0;                  // variable to manage if ASCOM move loop is running
float             motorSpeed = 1;                 // stepper speed in rpm (displaySpeed * 60)/stepsPerRevolution 
unsigned short    stepSpeed = 2;                  // display speed holder in steps/sec
unsigned short    stepSpeedCommand=16;            // Ascom command speed holder in steps/sec
long              distanceToGo = 0;               // focuser commanded position
long              currentPosition = 0;            // focuser current position
long              targetPosition = 0;             // focuser target position from serial command
long              runningStartPosition=0;          // focus start position for serial command to control speed ramping
int               runningInitialized = 0;         // running loop initialized
unsigned short    backlashState=0;                // backlash state controller 0= completed or not required, 1= backlash rackout, 2= backlash rackin to target position
bool              motorActive=false;              // track if motor drive output is active
bool              ascomFullStep=true;             // used to keep track of ascom commanded step mode.  Always start with full mode
bool              ascomModeSet=false;             // if ascom sets mode, manual buttons can not change

///////////////////////////
// Timer
///////////////////////////
long temperature_millis = 0;    // minute timer for updating readings 
long lastMotorRunMillis = 0;    // timer from when the last motor move was completed.
long buttonPressMillis = 0;     // timer for managing button press debounce rather than delay in program



#ifdef LCD1602Shield  // Code if LCD Shield installed
///////////////////////////
// LCD 1602 shield initialization and subroutines
///////////////////////////
/*
 * 1602 Shield uses the following pin assignments
 * D4-LCD D4
 * D5-LCD D5
 * D6-LCD D6
 * D7-LCD D7
 * D8-LCD Reset
 * D9-LCD Enable
 * D10-LCD BackLight
 * A0 - Buttons
 * Button readings have two different analog ranges depending on which version is purchased.  
 * Need to select using comments // which pin assignment works for your LCD shield in read_LCD_buttons() function
 */
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);    // select the pins used on the LCD panel

// define some values used by the panel and buttons
unsigned int lcd_key         = 0;
unsigned int adc_key_in      = 0;
unsigned int btnHoldCount    = 0;          // counter if button is being held
unsigned short flg_mode      = 0;          // control stepper mode Full or Half steppin
unsigned short flg_lcdlight  = 0;          // control display brightness
String topline = "PnPFocus        ";       // Standard top line display


#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

///////////////////////////
// LCD 1602 Subroutines and Functions
///////////////////////////

/////////////////////////////////////
// Display text at xposition, yposition and number of mS to delay after the text is written.
// Delay actually causes the program to stop and miss serial commands.  
// Use delay sparingly
/////////////////////////////////////
void displayText (int xpos, int ypos, String displayText,int delayPeriod)
{
    lcd.setCursor(xpos,ypos);             // set the LCD cursor   position 
    lcd.print(displayText);              // print message on the LCD
    delay (delayPeriod);
}   // end display Text

/////////////////////////////////////
// display the current motor step position counter
/////////////////////////////////////
void displayPosition()
{
      displayText (0, 1, "CP:     ", 0);
      displayText (3, 1,String(currentPosition),0);
}
/////////////////////////////////////
// display the ASCOM Target position
/////////////////////////////////////
void displayTarget()
{
        displayText (0, 0, "TP:     ",0);
        displayText (3, 0, String(targetPosition),0);
}

/////////////////////////////////////
// display current temperature sensor reading results
/////////////////////////////////////
void displayTemp (float temp)
{ displayText (9,1,"     ",0);       
  if ((temp < -50) || (temp > 50)){
        displayText(9,1,"T:N/A",0);
  } else {
        if (displayFahrenheit)
        {
          displayText(9,1,"T:"+ String(round(sensors.toFahrenheit(tempC))) + "F",0);   
        } else 
        {
          displayText(9,1,"T:"+ String(round(tempC)) + "C",0);   
        }
  } 
} // end displayTemp

/////////////////////////////////////
// turn ON drive power display indicator
/////////////////////////////////////
void displayStepperPowerOn ()
{
  if (myStepper.getStepMode() == FULLSTEP)
    displayText(15,1,"F",0);
  else
    displayText(15,1,"H",0);
}

/////////////////////////////////////
// turn OFF drive power display indicator
/////////////////////////////////////
void displayStepperPowerOff ()
{
  displayText(15,1," ",0);
}


/////////////////////////////////////
// display the button value diagnotics
// input is the unsigned In reading of pin A0
/////////////////////////////////////
void displayButtonValue(unsigned int buttonValue)
{
        displayText (0, 0, "BV:     ",0);
        displayText (3, 0, String(buttonValue),0);
}

int read_LCD_buttons()
{  // read the buttons
  
    adc_key_in = analogRead(0);                       // read the value from the sensor 
    //displayText (12, 0, String(adc_key_in), 0);     // used for troubleshooting
   
    // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
    // we add approx 50 to those values and check to see if we are close
    // We make this the 1st option for speed reasons since it will be the most likely result

    if (adc_key_in > 850) 
    {
      return btnNONE;                                   // fisrt check totally bypass all assessment if button is not pressed.
    }
    else // button is pressed
    {
      delay(2);                                         // switch debounce delay. Increase this delay if incorrect switch selections are returned.
      if (adc_key_in != analogRead(0)) return btnNONE;  // double checks the keypress. If the two readings are not equal after debounce delay, it tries again.
 
      // yes button still pressed after 10ms and button reading is still the same value execute the button
      // For V1.1 us this threshold
      /*
      if (adc_key_in < 50)   return btnRIGHT;  
      if (adc_key_in < 250)  return btnUP; 
      if (adc_key_in < 450)  return btnDOWN; 
      if (adc_key_in < 650)  return btnLEFT; 
      if (adc_key_in < 850)  return btnSELECT;  
      */
     // For V1.0 comment the other threshold and use the one below:
   
     if (adc_key_in < 50)   return btnRIGHT;//increase value
     if (adc_key_in < 195)  return btnUP;   //screen ON/OFF
     if (adc_key_in < 380)  return btnDOWN; //No Activity
     if (adc_key_in < 555)  return btnLEFT; //Decrease value
     if (adc_key_in < 790)  return btnSELECT; //Stop and save

    return btnNONE;                // when all others fail, return this.
        
  }
}  // end read_LCD_buttons

void buttonAction (int buttonCode)
{
  switch (buttonCode)
   {             
       case btnRIGHT:
         if (isRunning == 0) { 
            buttonSpeed(btnHoldCount);
            buttonMove(1);
            btnHoldCount++;
         }
         break;
       case btnLEFT:
         if (isRunning == 0) { 
            buttonSpeed(btnHoldCount);
            buttonMove(-1);
            btnHoldCount++;
         }
         break;    
       case btnUP:
             flg_lcdlight++;
             switch (flg_lcdlight){
              case 0:
                  break;       
              case 1:
                  setBrightness (0);
                  break;
              case 2:
                  setBrightness (1);
                  break;
              case 3:
                  setBrightness (5);
                  break;
              case 4:
                  setBrightness (20);
                  break;
              case 5:
                  setBrightness (200);
                  flg_lcdlight=0;
                  break;              
             }
             delay (300);    // note holding button down will cycle through the values in 1.5 seconds
             break;
       case btnDOWN:
            if (enableHalfStepMode)
            {
             if (ascomModeSet == false) 
               { 
                 if (myStepper.getStepMode()==FULLSTEP) 
                    changeStepMode(HALFSTEP);
                 else
                    changeStepMode(FULLSTEP);          
               }
               delay (500);    // note holding button down will cycle through the values in 1.5 seconds             
            }
                break;
       case btnSELECT:
             isRunning = 0; // STOP
             // EEPROM HERE
             EEPROMWritelong(0, currentPosition);
             EEPROM.write(4, myStepper.getStepMode());
             displayText (0, 0, "STOPPED & SAVED ", 1000);      
             displayText (0, 0, topline, 0); 
             displayTemp (tempC);  
             break;
       case btnNONE:
             break;
   }      
} // end buttonAction


/////////////////////////////////////
// button speed controller 
// input is number of steps taken so far with button pressed
/////////////////////////////////////
void buttonSpeed(unsigned int count)
{
  switch (count)
  {
    case 0 :
        setFocusStepSpeed(2);
        break;
    case 10:
        setFocusStepSpeed(10);
        break;
    case 50:
        setFocusStepSpeed(50);
        break;
    case 300:
        setFocusStepSpeed(100);
        break;
    case 700:
        setFocusStepSpeed(200);
        break;
  }
}

///////////////////////////////////////
// function to move motor by 1 position when button is pressed
// x input is 1 to increase and -1 to decrease current position
///////////////////////////////////////
void buttonMove(int x)
{
    currentPosition=positionChange(currentPosition, x);
    displayStepperPowerOn();                      // Turn on the motor active display tag
    displayPosition();                            // Display Current Position
    myStepper.step(x);
    lastMotorRunMillis=millis();                   // reset last motor movement timer
}

/////////////////////////////////////
// set the brightness of the display
// input 0-255
/////////////////////////////////////
void setBrightness (short bright)
{
  if(bright <0) bright = 0;
  if(bright >255) bright =255; 
  analogWrite(10, bright);
}

#endif  // LCD1602Shield


///////////////////////////
// Common Subroutines and Functions
///////////////////////////



//////////////////////////////////////////////////
// Temperature request actions to get value 
// Every 30 seconds initiate the temperature query
// Once conversion is completed, set value to temperature variable
//////////////////////////////////////////////////
void temperature_action ()                              
{ // This query basically runs in standalone mode when the ascom driver is not connected.  The Ascom Driver will query and reset temperature_millis much sooner than 30 seconds.
  if ((millis() - temperature_millis) > 28000)
  {   //request the temperatures
      sensors.requestTemperatures();                    // need to send a request to query the sensor buss
  }

  if ((millis() - temperature_millis) > 30000) 
  {//Update temperature variable every 30 seconds 
        readTemp();   
  }
}

//////////////////////////////////////////////////
//  Function reads and displays the temperature and resets the temperature timer
//  requires sensors.requestTempertures(); to be issued minimum 1 second before the read
//////////////////////////////////////////////////
void readTemp()                                         
{
      tempC = sensors.getTempCByIndex(0);      
      #ifdef LCD1602Shield  // Code if LCD Shield installed
        displayTemp (tempC);
      #endif  // LCD1602Shield
      temperature_millis=millis();                      // start counter from last temperature reporting  
}

///////////////////////////////////////////////////
//  This function will write a 4 byte (32bit) long to the eeprom at
//  the specified address to address + 3.
///////////////////////////////////////////////////
void EEPROMWritelong(int address, long value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      }

///////////////////////////////////////////////////
//  Read 4 bytes from EEProm memory location
///////////////////////////////////////////////////
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }

///////////////////////////////////////////////////
//  Calculate the motor speed(rpm) based on requestes
//  step speed and the stepper motor steps/revolution 
///////////////////////////////////////////////////
float calcMotorSpeed(unsigned short step_speed, int maxSteps)  // return motorSpeed in RPM
{ // step_speed is steps/sec
  float motor_speed;
  if (step_speed <1 ) step_speed = 1;               // make sures speed is positive and never 0
  motor_speed = (float)step_speed *60/maxSteps;     // Note arduino calculation default is int so have to tell it to do a float calculation
  return (motor_speed);
}



///////////////////////////////////////////////////
//  convert Hex string to long
///////////////////////////////////////////////////
long hexstr2long(char *line) {
  long ret = 0;

  ret = strtol(line, NULL, 16);
  return (ret);
}


////////////////////////////////////////////////////////////
// Process a Ascom serial command
////////////////////////////////////////////////////////////
void processCommand()
{
  int n = 0;  // counter for switched outputs
  
  MoonliteCommand_t command;
  switch (SerialProtocol.getCommand().commandID)
  {
    /*  if unit does not respond with correct unit version then it assumes a 2 unit version and program needs to respond properly
     *   Leaving unit 2 responses commented out for now in case someone wants to add a second ardumoto board for a 2 drive unit
    //  if unit does not respond with correct unit version then it assumes a 2 unit version and program needs to respond properly to unit 2 commands
    // ensure the program responds with version 13 for a single focus controller otherwise have to adapt with second unit responses.
    // a person could adapt this to use two ardumoto boards and no display to drive 2 focusers or possibly a focuser/rotatator, provided used ardumoto with two different
    // pin assignments.
    // start the unit 2 responses
    case ML_2C:
      // Initiate temperature convertion
      // Do nothing as temperatures are pulled every 5 seconds
      break;
    case ML_2FG:
      // Goto target position
      //Motor.goToTargetPosition();
      isRunning = 1;
      break;
    case ML_2FQ:
      // Motor stop movement
      //Motor.stopMovement();
      isRunning = 0;
      targetPosition = currentPosition;         // Need to reset target position = current position otherwise driver faults not reaching target
      break;
    case ML_2GB:
      // Set the Red Led backligth value
      // Dump value necessary to run the official moonlite software
      // LED backlight value, always return "00"
      // do nothing for now 
      break;
    case ML_2GC:
      // Return the temperature coefficient
      // are not using temp compensation
      SerialProtocol.setAnswer(2, 0x02);
      break;
    case ML_2GD:
      // Return the current motor speed
      n=0;         // reset n before using
      stepSpeed = myStepper.getStepSpeed();
      if (stepSpeed<=16) n++;
      if (stepSpeed<=32) n++;
      if (stepSpeed<=63) n++;
      if (stepSpeed<=125) n++;
      if (stepSpeed>125) n++;
      switch (n)
        {
          case 0:
            SerialProtocol.setAnswer(2, 0x20);
            break;
          case 1:
            SerialProtocol.setAnswer(2, 0x10);
            break;          
          case 2:
            SerialProtocol.setAnswer(2, 0x08);
            break; 
          case 3:
            SerialProtocol.setAnswer(2, 0x04);
            break; 
          case 4:
            SerialProtocol.setAnswer(2, 0x02);
            break; 
           default:
           break; 
        }
      break;
    case ML_2GH:
      // Return the current stepping mode (half or full step)
      SerialProtocol.setAnswer(2, (long)(0x00));
      break;
    case ML_2GI:
      // get if the motor is moving or not
      SerialProtocol.setAnswer(2, (long)(isRunning ? 0x01 : 0x00));
      break;
    case ML_2GN:
      // Get the target position
      SerialProtocol.setAnswer(4, (long)(targetPosition));
      break;
    case ML_2GP:
      // Return the current position
      SerialProtocol.setAnswer(4, (long)(currentPosition));
      break;
    case ML_2GT:
      // Return the temperature
      // Temperature update every 5 seconds so just report latest temperature *2 because of 2's complement
      SerialProtocol.setAnswer(4, (long)(tempC*2));
      break;
    case ML_2GV:
      // Get the version of the firmware
      //SerialProtocol.setAnswer(2, (long)(0x02));
      break;
    case ML_2SC:
      // Set the temperature coefficient
      //Motor.setTemperatureCompensationCoefficient(SerialProtocol.getCommand().parameter);
      // dont use so don't respond
      break;
    case ML_2SD:
      // Set the motor speed
      switch (SerialProtocol.getCommand().parameter)
      {
        case 0x02: // 250 Steps/sec
          stepSpeedCommand=250;         // Speed will be confirmed and set before eacy move 
         break;
        case 0x04: // 125 Steps/sec  //
          stepSpeedCommand=125;
          break;
        case 0x08:  // 63 Steps/sec
          stepSpeedCommand=63;
          break;
        case 0x10:  // 32 Steps/sec
          stepSpeedCommand=32;
          break;
        case 0x20:  // 16 Steps/sec
          stepSpeedCommand=16;
          break;
      }
      break;
    case ML_2SF:
      // Set the stepping mode to full step
      // Do nothing - only uses full steps
      break;
    case ML_2SH:
      // Set the stepping mode to half step
      // Do nothing - only uses full steps
      break;
    case ML_2SN:
      // Set the target position
      targetPosition = (SerialProtocol.getCommand().parameter);
      break;
    case ML_2SP:
      // Set the current motor position
      currentPosition = (SerialProtocol.getCommand().parameter);
      break;
    case ML_2PLUS:
      // Activate temperature compensation focusing
      // Do nothing - did not implement temperature compensation
      break;
    case ML_2MINUS:
      // Disable temperature compensation focusing
      // Do nothing - did not implement temperature compensation
      break;
    case ML_2PO:
      // Temperature calibration
      // Do nothing - did not implement temperature compensation
      break;
    */ 
    // start the unit 1 responses
    case ML_C:
      // Initiate temperature convertion
      // only perform if focus move is not initiated
      if (isRunning == 0) //
      {
           sensors.requestTemperatures();     
      }
      break;
    case ML_FG:
      // Goto target position
      isRunning = 1;
      break;
    case ML_FQ:
      // Motor stop movement                    Issued when Ascom driver disconnects
      isRunning = 0;
      targetPosition = currentPosition;         // Need to reset target position = current position otherwise driver faults not reaching target
      #ifdef LCD1602Shield  
          displayText (0, 0, topline, 0);       // remove TP and Display the topline on the display
      #endif  // LCD1602Shield
      EEPROMWritelong(0, currentPosition);      // write the current position to EEProm memory
      EEPROM.write(4, myStepper.getStepMode());
      ascomModeSet= false;                      // if release the step mode lock
      break;
    case ML_GB:
      // Set the Red Led backligth value
      // Dump value necessary to run the official moonlite software
      // LED backlight value, always return "00"
      //  Do nothing for now but will be something like setBrightness(hexstr2long(SerialProtocol.getCommand().parameter));
      
      break;
    case ML_GC:
      // Return the temperature coefficient
      // are not using temp compensation
      SerialProtocol.setAnswer(2, 0x02);
      break;
    case ML_GD:
      // Return the current motor speed
      n=0;         // reset n before using
      stepSpeed = myStepper.getStepSpeed();
      if (stepSpeed<=16) n++;
      if (stepSpeed<=32) n++;
      if (stepSpeed<=63) n++;
      if (stepSpeed<=125) n++;
      if (stepSpeed>125) n++;
      switch (n){
          case 0:
            SerialProtocol.setAnswer(2, 0x20);
            break;
          case 1:
            SerialProtocol.setAnswer(2, 0x10);
            break;          
          case 2:
            SerialProtocol.setAnswer(2, 0x08);
            break; 
          case 3:
            SerialProtocol.setAnswer(2, 0x04);
            break; 
          case 4:
            SerialProtocol.setAnswer(2, 0x02);
            break; 
          default:
           break; 
        }  
      break;
    case ML_GH:
      // Return the current stepping mode (half or full step)
      // FF for half 00 for Full
      SerialProtocol.setAnswer(2, (long)((myStepper.getStepMode()==FULLSTEP)? 0x00 : 0xFF));
      break;
    case ML_GI:
      // get if the motor is moving or not
      SerialProtocol.setAnswer(2, (long)(isRunning ? 0x01 : 0x00));
      break;
    case ML_GN:
      // Get the target position
      SerialProtocol.setAnswer(4, (long)(targetPosition));
      break;
    case ML_GP:
      // Return the current position
      SerialProtocol.setAnswer(4, (long)(currentPosition));
      break;
    case ML_GT:
      // Return the temperature
      // only perform temperature query if not running and then report latest temperature *2 because of 2's complement
      if (isRunning == 0) //
      {
        if (sensors.isConversionComplete() == true) // Test to be sure sensor read is finished.
        {
          readTemp();       
        } 
        if (displayFahrenheit)
        {
          SerialProtocol.setAnswer(4, (long)(sensors.toFahrenheit(tempC) *2));
        }else
        {
          SerialProtocol.setAnswer(4, (long)(tempC *2));
        }
      }
      break;
    case ML_GV:
      // Get the version of the firmware
      // Value needs to alwasy be 0x13 for moonlite DRO Ascom Driver to work properly as single focuser controller for the correct command codes....  DO NOT CHANGE
      // Moonlite used 23 for the dual DRO controller.  
      // Strange that even if value returned is 0x13 the DRO still thinks the controller is a dual head unit. Not quite sure how or why this exists.  As soon as a person connects
      // the moonlite focuser 
       SerialProtocol.setAnswer(2, 0x13);
      break;
    case ML_SC:
      // Set the temperature coefficient where number of steps per degree C.
      // confirmed the Moonlite does not send this command when using any temperature compensation enabled  Therefore do nothing
      break;
    case ML_SD:
      // Set the motor speed
      switch (SerialProtocol.getCommand().parameter)
      {
        case 0x02: // 250 Steps/sec
          stepSpeedCommand=250;         // Speed will be confirmed and set before eacy move 
         break;
        case 0x04: // 125 Steps/sec 
          stepSpeedCommand=125;
          break;
        case 0x08:  // 63 Steps/sec
          stepSpeedCommand=63;
          break;
        case 0x10:  // 32 Steps/sec
          stepSpeedCommand=32;
          break;
        case 0x20:  // 16 Steps/sec
          stepSpeedCommand=16;
          break;
      }
      break;
    case ML_SF:
      // Set the stepping mode to full step. 
      changeStepMode(FULLSTEP);
      ascomFullStep= true;
      ascomModeSet= true;       // if ascom sets mode, prevent button change control
      
      break;
    case ML_SH:
      // Set the stepping mode to half step
      if (enableHalfStepMode)
      {
        changeStepMode (HALFSTEP);
        ascomFullStep=false;
        ascomModeSet= true;       // if ascom sets mode, prevent button change control
      } else
      {
        // Halfstep disabled, therefore always have to ensure Full step is set if half step is selected.
        changeStepMode(FULLSTEP);
        ascomFullStep= true;
        ascomModeSet= true;       // if ascom sets mode, prevent button change control
      }
      break;
    case ML_SN:
      // Set the target position
      targetPosition = SerialProtocol.getCommand().parameter;
      #ifdef LCD1602Shield  // Code if LCD Shield installed
        displayTarget();
      #endif  // LCD1602Shield
      break;
    case ML_SP:
      // Set the current motor position
      currentPosition = SerialProtocol.getCommand().parameter;
      #ifdef LCD1602Shield  // Code if LCD Shield installed
        displayPosition();
      #endif  // LCD1602Shield
      break;
    case ML_PLUS:
      // Activate temperature compensation focusing
      // Confirmed the Moonlite DRO driver does not send this command for temperature compensation.  It just sends new target position commands as required
      // Do Nothing
      break;
    case ML_MINUS:
      // Disable temperature compensation focusing
      // Confirmed the Moonlite DRO driver does not send this command for temperature compensation.  It just sends new target position commands as required
      // Do Nothing
      break;
    case ML_PO:
      // Temperature calibration
      // Do nothing - did not implement temperature compensation
      break;
    default:
    break;
  }
}

///////////////////////////////////////////////////////////////
//  Position Rollover check and reset to zero
//  input current position and amount of change 
//  word variables will automatically roll over if they exceed 65535
///////////////////////////////////////////////////////////////
word positionChange (word pos, word change)
{   
    pos =  pos + change;
    return (pos);
}


//////////////////////////////////////////////////////
//  Process a move to target request from Ascom command
//////////////////////////////////////////////////////
void moveToTarget ()
{
  if (runningInitialized == 0)
  {
          runningStartPosition = currentPosition;                     // set start of move to target to control starting speed.
          /*
          if (ascomFullStep && myStepper.getStepMode()!= FULLSTEP )
          {
            changeStepMode(FULLSTEP);
          }
          if (!ascomFullStep && myStepper.getStepMode()!= HALFSTEP )
          {
            changeStepMode(HALFSTEP);
          }
          */
          if ( (targetPosition - currentPosition) > 5  && backlashValue!=0 )  // using 5 steps as the threshold to apply backlash compensation.
                                                                              // Temperature outward moves should not invoke backlash compensation adjustment
          {
            backlashState=1;                                          // Backlash compensation condition exists start rack out
          }
          #ifdef LCD1602Shield                                        // Code if LCD Shield installed
            displayStepperPowerOn();                                  // Turn on the motor active display tag
            displayText(8,0,"@"+String(stepSpeedCommand)+"S/s",0);    // Turn on the motor active display tag
            displayPosition();                                        // Display Current Position
          #endif  // LCD1602Shield
          runningInitialized =1;                                      // running actions are not initiallized and ready to loop.
  }////////////////// end of Running intialization
  
  switch (backlashState)
  {
    case 0:
      // No backlash compensation
      distanceToGo =  targetPosition - currentPosition;
      runningSpeedChange( runningStartPosition,currentPosition, distanceToGo); 
      break;
    case 1:
      // backlash compensation rackout
       distanceToGo = targetPosition + (ascomFullStep? backlashValue:backlashValue*2) - currentPosition;
       runningSpeedChange( runningStartPosition,currentPosition, distanceToGo); 
      break;
    case 2:
      // backlash rack out has been reached now perform rack in
      distanceToGo = targetPosition - currentPosition;
      runningSpeedChange( runningStartPosition,currentPosition, distanceToGo); 
      break;
  }
  
  if (distanceToGo > 0)
  {
    currentPosition=positionChange(currentPosition, 1);       
    #ifdef LCD1602Shield                                              // Code if LCD Shield installed
      displayPosition();                                              // Display Current Position
    #endif  // LCD1602Shield 
    myStepper.step(1);
    lastMotorRunMillis=millis();
  } else 
  {
    if (distanceToGo < 0) 
    {
      currentPosition=positionChange(currentPosition, -1);
      #ifdef LCD1602Shield  // Code if LCD Shield installed
        displayPosition();                            // Display Current Position
      #endif  // LCD1602Shield
      myStepper.step(-1);
      lastMotorRunMillis=millis();
    } else 
    { // target point reached so evalutate if running can be stopped
       switch (backlashState)
       {
        case 0:
          // no backlash to reset running parameters
          isRunning = 0;
          runningInitialized = 0;
          backlashState = 0;
          break;
        case 1:
          // backlash running and have reached rackout position and need to rack in now
          delay (300);                                // pause for 0.3 seconds to reverse direction
          runningStartPosition = currentPosition;     // set start of move to target to control starting speed.
          backlashState = 0;                          // Now start rackin to final target position
          break;
       }
    }
  }
}

//////////////////////////////////////////////////////
//  afterMotorRunChecks routine
//  input total millis since last motor movement.
//////////////////////////////////////////////////////
void motorIdleActions (long motorIdleMillis)
{
  short action = 0;       
  if (motorIdleMillis > 600  && motorIdleMillis < 800) action = 1;     // button not pressed actions
  if (motorIdleMillis > 2000 && motorIdleMillis < 2500) action = 2;   // 5 second actions
  // don't need any end time reset to zero because every assessment starts at 0
  switch (action)
  { case 0:
      break;
    case 1:                             // button not pressed actions
      #ifdef LCD1602Shield              // Compile and perform following code if LCD Shield installed
         btnHoldCount = 0;                 // need to clear out the button hold counter cause it was released
         displayText(8,0,"        ",0);    // Clear out the speed display as no longer moving by button press
      #endif  // LCD1602Shield
      break;
    case 2:                             // 5 second actions
      myStepper.release();              // disable output after 5 seconds from last motor move
      #ifdef LCD1602Shield              // Compile and perform following code if LCD Shield installed
         displayStepperPowerOff();      // Turn on the motor active display tag
      #endif  // LCD1602Shield
      break;
  }
}


//////////////////////////////////////////////////////
//  changeStepMode routine
//  input FULLSTEP 1, HALFSTEP 2 defined in Ardumoto.h
//////////////////////////////////////////////////////
void changeStepMode (short mode)
{if (mode != myStepper.getStepMode())
  {
    switch (mode)
    {  
      case FULLSTEP:
        myStepper.setStepMode(FULLSTEP);
        currentPosition=positionChange(currentPosition, -(currentPosition/2));
        #ifdef LCD1602Shield                                        // Code if LCD Shield installed
          displayPosition();                                        // Display Current Position
          displayText(0,0,"FullStep",0); 
        #endif  // LCD1602Shield
        break;
      case HALFSTEP:
        myStepper.setStepMode(HALFSTEP);
        currentPosition=positionChange(currentPosition, currentPosition);
        #ifdef LCD1602Shield                                        // Code if LCD Shield installed
          displayText(0,0,"HalfStep",0); 
          displayPosition();                                        // Display Current Position
        #endif  // LCD1602Shield
        break;
    }
  }
}


//////////////////////////////////////////////////////
//  runningSpeedChange
//  Changes speed commands to ensure soft start and setting speeds to enable ramped speed changes
//  input start position(sp), current position(cp), end position(ep)
//////////////////////////////////////////////////////
void runningSpeedChange(long sp, long cp, long dist)
{ short start ;                  // for evaluating distance from start or end  
  // softStart speed change
  start = abs(cp-sp);
  if (start <200)
  { 
    switch (start)
    {
      case 0:
        setFocusStepSpeed(5);             //start with 5 steps/sec
        break;
      case 5:
        setFocusStepSpeed(16);             //start with 16 steps/sec
        break;
      case 20:
        if(stepSpeedCommand > 30) setFocusStepSpeed(32);
        break;
      case 50:
        if(stepSpeedCommand > 60) setFocusStepSpeed(63);
        break;
      case 100:
        if(stepSpeedCommand > 100) setFocusStepSpeed(125);
        break;  
      case 195:
        setFocusStepSpeed(stepSpeedCommand);
        break;          
    break;  
    }
  }
  // settling speed change
  if (abs(dist) < 200)
  {
    switch (abs(dist))
    {
      case 10:
        setFocusStepSpeed(5);  
        break;
      case 30:
        setFocusStepSpeed(16);
        break;
      case 60:
        if(stepSpeedCommand > 30) setFocusStepSpeed(32);
        break; 
      case 120:
        if(stepSpeedCommand > 60) setFocusStepSpeed(63);
        break;       
      case 199:
        if(stepSpeedCommand > 100) setFocusStepSpeed(125);
        break; 
     break;  
    }
  }
}


//////////////////////////////////////////////////////
//  setFocusStepSpeed
//  program focus step speed changes and display 
//  input stepSpeed as steps/second
//////////////////////////////////////////////////////
void setFocusStepSpeed(unsigned short stepSpeed)
{
  myStepper.setSpeed (calcMotorSpeed(stepSpeed,stepsPerRevolution));   
  #ifdef LCD1602Shield                                        // Compile and perform following code if LCD Shield installed
     displayText(8,0," @" + String(stepSpeed) + "S/s ",0);    // Turn on the motor active display tag
  #endif  // LCD1602Shield
}



///////////////////////////
// Setup 
///////////////////////////

void setup()
{ 
  //Setup Ardumoto board
  sensors.begin();                                             // Start temperature sensor as it takes the longest
  SerialProtocol.init(9600);                                   // Setup serial port monitor
  switch (EEPROM.read(4))                                      // Read the mode value from non-volitile memory
  {
    case FULLSTEP:
      myStepper.setStepMode(FULLSTEP);
      break;
    case HALFSTEP:
      myStepper.setStepMode(HALFSTEP);
      break;
    default:                                                    // need a default to fullStep in case memory value hasn't been recorded
      myStepper.setStepMode(FULLSTEP);
      break;      
  }
  currentPosition=EEPROMReadlong(0);                           //Setup EEProm to store and retrieve last focuser position
  temperature_millis=millis();                                 // start timer for 30 second update timer
  lastMotorRunMillis=millis();                                 // start timer for motor turn disable
  #ifdef LCD1602Shield  // Code if LCD Shield installed
     lcd.begin(16, 2);                                         // start the LCD library
     setBrightness (userStartBrightness);                      // start with user selected brightness at top of code
     displayText (0, 0, topline, 0);
     displayText (8,0,("S/R:"+String(stepsPerRevolution)),2000);
     displayPosition();
     buttonPressMillis=millis();                               // start timer for motor turn disable
  #endif  // LCD1602Shield
}

///////////////////////////
// Main Loop 
///////////////////////////

void loop()
{
    temperature_action ();                    // perform temperature action if required
    #ifdef LCD1602Shield                      // Compile and perform following code if LCD Shield installed
      //displayText (0, 0, String(millis()), 0);            // only used for troubleshooting to see if programs is halted
      lcd_key = read_LCD_buttons();           // read the buttons
      buttonAction(lcd_key);                  // action on button press
    #endif  // LCD1602Shield
    if (isRunning) {                          // perform target moves if required
      moveToTarget();
    }else {
      if (runningInitialized !=0) runningInitialized = 0;     // Reset runningInitialized if the case of an abort;
    }
    motorIdleActions((millis() - lastMotorRunMillis));
    SerialProtocol.Manage();
    if (SerialProtocol.isNewCommandAvailable()){
      processCommand();
    }
} // end loop

/***********************************************************************************
 * Reference
 */

// Moonlite-compatible stepper controller
//
//1  2 3 4 5 6 7 8
//: C #             N/A         Initiate a temperature conversion; the conversion process takes a maximum of 750 milliseconds. The value returned by the :GT# command will not be valid until the conversion process completes.
//: F G #           N/A         Go to the new position as set by the ":SNYYYY#" command.
//: F Q #           N/A         Immediately stop any focus motor movement.
//: G C #           XX#         Returns the temperature coefficient where XX is a two-digit signed (2’s complement) hex number.  (not issued in F/W V13)
//: G D #           XX#         Returns the current stepping delay where XX is a two-digit unsigned hex number. See the :SD# command for a list of possible return values.
//: G H #           00# OR FF#  Returns "FF#" if the focus motor is half-stepped otherwise return "00#"
//: G I #           00# OR 01#  Returns "00#" if the focus motor is not moving, otherwise return "01#"
//: G N #           YYYY#         Returns the new position previously set by a ":SNYYYY" command where YYYY is a four-digit unsigned hex number.
//: G P #           YYYY#         Returns the current position where YYYY is a four-digit unsigned hex number.
//: G T #           YYYY#         Returns the current temperature where YYYY is a four-digit signed (2’s complement) hex number.
//: G V #           DD#         Get the version of the firmware as a two-digit decimal number where the first digit is the major version number, and the second digit is the minor version number.
//: S C X X #       N/A         Set the new temperature coefficient where XX is a two-digit, signed (2’s complement) hex number.
//: S D X X #       N/A         Set the new stepping delay where XX is a two-digit, unsigned hex number. Valid values to send are 02, 04, 08, 10 and 20, which correspond to a stepping delay of 250, 125, 63, 32 and 16 steps per second respectively.
//: S F #           N/A         Set full-step mode.
//: S H #           N/A         Set half-step mode.
//: S N Y Y Y Y #   N/A         Set the new position where YYYY is a four-digit unsigned hex number.
//: S P Y Y Y Y #   N/A         Set the current position where YYYY is a four-digit unsigned hex number.
//: + #             N/A         Activate temperature compensation focusing.   (not issued in F/W V13)
//: - #             N/A         Disable temperature compensation focusing.    (not issued in F/W V13)
//: P O X X #       N/A         Temperature calibration offset, XX is a two-digit signed hex number, in half degree increments.
