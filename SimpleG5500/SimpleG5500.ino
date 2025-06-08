/*
  SimpleSat Rotor Control Program  -  73 de W9KE Tom Doyle
  November 2011

  This program was written for the Arduino boards. It has been tested on the
  Arduino UNO and Mega2560 boards.

  MODIFIED BY RAYDEL ABREU CM2ESP. ADDED MORE GS232 COMMANDS AND FUNCTIONS
  SERIAL LCD DISPLAY DISABLED AND REPLACED BY 7 SEGMENT CONTROLLER

  The Arduino usb port is set to 9600 baud and is used for receiving
  data from the tracking program in GS232 format.
  In SatPC32 set the rotor interface to Yaesu_GS-232.

  These pin assignments can be changed
  by changting the assignment statements below.
  G-5500 analog azimuth to Arduino pin A0
  G-5500 analog elevation to Arduino pin A1
  Use a small signal transistor switch or small reed relay for these connections
  G-5500 elevation rotor up to Arduino pin 8
  G-5500 elevation rotor down to Arduino pin 9
  G-5500 azimuth rotor left to Arduino pin 10
  G-5500 azimuth rotor right to Arduino pin 11

  The Arduino resets when a connection is established between the computer
  and the rotor controller. This is a characteristic of the board. It makes
  programming the chip easier. It is not a problem but is something you
  should be aware of.

  The program is set up for use with a Yaesu G5500 rotor which has a max
  azimuth of 450 degrees and a max elevation of 180 degrees. The controller
  will accept headings within this range. If you wish to limit the rotation
  to 360 and/or limit the elevation to 90 set up SatPC32 to limit the rotation
  in the rotor setup menu. You should not have to change the rotor controller.

            - For additional information check -

      http://www.tomdoyle.org/SimpleSatRotorController/
*/

/*
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    INCLUDING BUT NOT LIMITED TO THE'WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
    PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR 'COPYRIGHT HOLDERS BE
    LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
    OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/
boolean DEBUG_FLAG = false;
// ------------------------------------------------------------
// ---------- you may wish to adjust these values -------------
// ------------------------------------------------------------

// A/D converter parameters
/*
   AFTER you have adjusted your G-5500 control box as per the manual
   adjust the next 4 parameters. The settings interact a bit so you may have
   to go back and forth a few times. Remember the G-5500 rotors are not all that
   accurate (within 4 degrees at best) so try not to get too compulsive when
   making these adjustments.
*/

long _azAdZeroOffset   =   0;   // adjust to zero out lcd az reading when control box az = 0
long _elAdZeroOffset   =   107;   // adjust to zero out lcd el reading when control box el = 0

/*
    10 bit A/D converters in the Arduino have a max value of 1023
    for the azimuth the A/D value of 1023 should correspond to 450 degrees
    for the elevation the A/D value of 1023 should correspond to 180 degrees
    integer math is used so the scale value is multiplied by 100 to maintain accuracy
    the scale factor should be 100 * (1023 / 450) for the azimuth
    the scale factor should be 100 * (1023 / 180) for the elevation
*/

long _azScaleFactor =  262;  //  adjust as needed
long _elScaleFactor =  568;  //  adjust as needed

// pins
const byte _azimuthInputPin = A0;   // azimuth analog signal from G5500
const byte _elevationInputPin = A1; // elevation analog signal from G5500
const byte _G5500UpPin = 6;        // elevation rotor up control line
const byte _G5500DownPin = 7;      // elevation rotor down control line
const byte _G5500LeftPin = 5;      // azimuth rotor left control line
const byte _G5500RightPin = 4;     // azimuth rotor right control line

// take care if you lower this value -  wear or dirt on the pots in your rotors
// or A/D converter jitter may cause hunting if the value is too low.
long _closeEnough = 100;   // tolerance for az-el match in rotor move in degrees * 100

// ------------------------------------------------------------
// ------ values from here down should not need adjusting -----
// ------------------------------------------------------------

// rotor
long _maxRotorAzimuth = 45000L;  // maximum rotor azimuth in degrees * 100
long _maxRotorElevation = 18000L; // maximum rotor elevation in degrees * 100

long _AZsensorValue = 0L;      // current rotor azimuth raw ADC value
long _ELsensorValue = 0L;      // current rotor elevation raw ADC value
long _rotorAzimuth = 0L;       // current rotor azimuth in degrees * 100
long _rotorElevation = 0L;     // current rotor elevation in degrees * 100
long _azimuthTemp = 0L;        // used for gs232 azimuth decoding
long _elevationTemp = 0L;      // used for gs232 elevation decoding
long _newAzimuth = 0L;         // new azimuth for rotor move
long _newElevation = 0L;       // new elevation for rotor move
long _previousRotorAzimuth = 0L;       // previous rotor azimuth in degrees * 100
long _previousRotorElevation = 0L;     // previous rotor azimuth in degrees * 100

unsigned long rtcCurrent = 0UL;                 // rtc initialization
unsigned long _rtcLastDisplayUpdate = 0UL;      // rtc at start of last loop
unsigned long _rtcLastRotorUpdate = 0UL;        // rtc at start of last loop
unsigned long _displayUpdateInterval = 1000UL;   // display update interval in mS
unsigned long _rotorMoveUpdateInterval = 100UL; // rotor move check interval in mS

boolean _gs232WActice = false;  // gs232 W command in process
int _gs232AzElIndex = 0;        // position in gs232 Az El sequence
long _gs232Azimuth = 0;          // gs232 Azimuth value
long _gs232Elevation = 0;        // gs232 Elevation value
boolean _azimuthMove = false;    // azimuth move needed
boolean _elevationMove = false;  // elevation move needed
boolean _manualMove = false;     // to indicate we are moving manually
boolean _lockMode = false;       // lock and block any movement from external or manual input
boolean _remoteAzmMove = false;  // remote manual input command to turn Azm received
boolean _remoteElevMove = false; // remote manual input command to turn Elev received

// CDC Buffers for text comm
char character;    // Incoming character one at the time
String incomm;

//Reading Analogue Averague
const u8 avgMany = 20;    //how many times average value
const int avgDelay = 100; //adc reading delay in uS

//Reset Function
void(* resetFunc) (void) = 0;

//For the 7-segment display controller
#include <Wire.h>
#include <TM1650.h>
TM1650 d;
boolean displayCycle = false;
byte button = 0x00;

//Persistent data config
#include <EEPROM.h>

//
// run once at reset
//
void setup()
{
  //Init display
  Wire.begin(); //Join the bus as master
  d.init();
  delay(500);
  // Display init
  d.displayOn();
  d.setBrightness(TM1650_MAX_BRIGHT);
  d.displayString("boot");

  // initialize rotor control pins as outputs
  pinMode(_G5500UpPin, OUTPUT);
  pinMode(_G5500DownPin, OUTPUT);
  pinMode(_G5500LeftPin, OUTPUT);
  pinMode(_G5500RightPin, OUTPUT);

  // initialize LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);

  // set all the rotor control outputs low
  digitalWrite(_G5500UpPin, LOW);
  digitalWrite(_G5500DownPin, LOW);
  digitalWrite(_G5500LeftPin, LOW);
  digitalWrite(_G5500RightPin, LOW);

  // set LED_BUILTIN off
  digitalWrite(LED_BUILTIN, LOW);

  //Read Persistent Config from EEPROM
  readPersistent();

  //Notify boot ready on Serial
  // initialize serial ports:
  Serial.begin(9600);  // control
  Serial.setTimeout(50);
  Serial.println("   W9KE V1.7    ");
  Serial.println("   ==READY==    ");
  delay(1000);
  Serial.flush();

  //Boot and config ready
  // Display pause and reset after boot
  delay(2000);
  d.displayString("    ");

  // set up rotor lcd display values
  readAzimuth(); // get current azimuth from G-5500
  _previousRotorAzimuth = _rotorAzimuth + 1000;
  readElevation(); // get current elevation from G-5500
  _previousRotorElevation = _rotorElevation + 1000;
}


//
// main program loop
//
void loop()
{
  // check for serial data
  if(Serial)
  {
  if (Serial.available() > 0)
  {
    //character = Serial.read();
    incomm=Serial.readStringUntil('\r');
    decodeGS232();
    //Serial.flush();
  }  
  }

  // read TM1650buttons
  CheckButtons();

  // get current rtc value
  rtcCurrent = millis();

  // check for rtc overflow - skip this cycle if overflow
  if (rtcCurrent > _rtcLastDisplayUpdate) // overflow if not true    _rotorMoveUpdateInterval
  {
    // update rotor movement if necessary
    if (rtcCurrent - _rtcLastRotorUpdate > _rotorMoveUpdateInterval)
    {
      _rtcLastRotorUpdate = rtcCurrent; // reset rotor move timer base

      // READ POSITION
      readAzimuth(); // get current azimuth from G-5500
      readElevation(); // get current elevation from G-5500

      if (!_manualMove && !_lockMode && !_remoteAzmMove && !_remoteElevMove) //if we are not in Manual Movement Mode nor Lock Mode then run motors
      {
        // AZIMUTH
        // see if azimuth move is required
        if ( (abs(_rotorAzimuth - _newAzimuth) > _closeEnough) && _azimuthMove )
        {
          updateAzimuthMove();
        }
        else  // no move required - turn off azimuth rotor
        {
          digitalWrite(_G5500LeftPin, LOW);
          digitalWrite(_G5500RightPin, LOW);
          _azimuthMove = false;
        }

        // ELEVATION
        // see if aelevation move is required
        if ( abs(_rotorElevation - _newElevation) > _closeEnough && _elevationMove ) // move required
        {
          updateElevationMove();
        }
        else  // no move required - turn off elevation rotor
        {
          digitalWrite(_G5500UpPin, LOW);
          digitalWrite(_G5500DownPin, LOW);
          _elevationMove = false;
        }
      } // end of manual and lock mode detection
    }   // end of update rotor move


    // If there is movement turn LED_BUILTIN ON to notify
    if ((_elevationMove || _azimuthMove || _manualMove || _remoteAzmMove || _remoteElevMove) && !_lockMode)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      d.setDot(2, true);
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
      d.setDot(2, false);
    }

    // update display if necessary
    if (rtcCurrent - _rtcLastDisplayUpdate > _displayUpdateInterval)
    {
      // update rtcLast
      _rtcLastDisplayUpdate = rtcCurrent;  // reset display update counter base
      displayAzEl();
    }
  }
  else // rtc overflow - just in case
  {
    // update rtcLast
    _rtcLastDisplayUpdate = rtcCurrent;
  }
delay(5);  
}


//
// update elevation rotor move
//
void updateElevationMove()
{
  // calculate rotor move
  long rotorMoveEl = _newElevation - _rotorElevation;

  if (rotorMoveEl > 0)
  {
    //elRotorMovement = "  U ";
    digitalWrite(_G5500DownPin, LOW);
    digitalWrite(_G5500UpPin, HIGH);
  }
  else
  {
    if (rotorMoveEl < 0)
    {
      //elRotorMovement = "  D ";
      digitalWrite(_G5500UpPin, LOW);
      digitalWrite(_G5500DownPin, HIGH);
    }
  }
}


//
// update azimuth rotor move
//
void updateAzimuthMove()
{
  // calculate rotor move
  long rotorMoveAz = _newAzimuth - _rotorAzimuth;
  // adjust move if necessary
  if (rotorMoveAz > 18000) // adjust move if > 180 degrees
  {
    rotorMoveAz = rotorMoveAz - 180;
  }
  else
  {
    if (rotorMoveAz < -18000) // adjust move if < -180 degrees
    {
      rotorMoveAz = rotorMoveAz + 18000;
    }
  }

  if (rotorMoveAz > 0)
  {
    //azRotorMovement = "  R ";
    digitalWrite(_G5500LeftPin, LOW);
    digitalWrite(_G5500RightPin, HIGH);
  }
  else
  {
    if (rotorMoveAz < 0)
    {
      //azRotorMovement = "  L ";
      digitalWrite(_G5500RightPin, LOW);
      digitalWrite(_G5500LeftPin, HIGH);
    }
  }
}


//
// read azimuth from G5500
//
void readElevation()
{
  _ELsensorValue = analogRead(_elevationInputPin);
  
  for (u8 i = 0; i < avgMany; i++)
  {
    _ELsensorValue += analogRead(_elevationInputPin);
    delayMicroseconds(avgDelay);
  }
  _ELsensorValue = (long)((float)_ELsensorValue / (float)(avgMany));

  _rotorElevation = ((_ELsensorValue * 10000) / _elScaleFactor) - _elAdZeroOffset;
}


//
// read azimuth from G5500
//
void readAzimuth()
{
  _AZsensorValue = analogRead(_azimuthInputPin);
  
  for (u8 i = 0; i < avgMany; i++)
  {
    _AZsensorValue += analogRead(_azimuthInputPin);
    delayMicroseconds(avgDelay);
  }
  _AZsensorValue = (long)((float)_AZsensorValue / (float)(avgMany));
  //Serial.println(_AZsensorValue);

  _rotorAzimuth = ((_AZsensorValue * 10000) / _azScaleFactor) - _azAdZeroOffset;
}


//
// decode gs232 commands
//
void decodeGS232()
{
  character=(incomm.length()>0)?incomm.charAt(0):0x00;
  switch (character)
  {
    case 'w':  // gs232 W command
    case 'W':
      {
        {
          //Validate W sentence
          if(incomm.length()==8 && incomm.charAt(4)==' ')
          {
          _azimuthTemp = incomm.substring(1,4).toInt();
          _elevationTemp = incomm.substring(5).toInt();
          _newAzimuth = _azimuthTemp * 100;
          _newElevation = _elevationTemp * 100;
          _azimuthMove = true;
          _elevationMove = true;
          incomm = "";
          character = 0x00;
          }
        }
        break;
      }

    case 'c':
    case 'C':
      {
        char buf[11];
        sprintf(buf, "+%04d+%04d\r", (int)(_rotorAzimuth / 100.0f), (int)(_rotorElevation / 100.0f));
        Serial.print(buf);
        break;
      }

    case 'b':
    case 'B':
      {
        char buf[6];
        sprintf(buf, "+%04d\r", (int)(_rotorElevation / 100.0f));
        Serial.print(buf);
        break;
      }

    case 'u':
    case 'U':
      {
        _gs232WActice = false;
        _elevationMove = false;
        _azimuthMove = false;
        _manualMove = true;
        _remoteAzmMove = false;
        _remoteElevMove = true;
        // set the rotor control outputs to up
        digitalWrite(_G5500UpPin, HIGH);
        digitalWrite(_G5500DownPin, LOW);
        digitalWrite(_G5500LeftPin, LOW);
        digitalWrite(_G5500RightPin, LOW);
        break;
      }

    case 'd':
    case 'D':
      {
        _gs232WActice = false;
        _elevationMove = false;
        _azimuthMove = false;
        _manualMove = true;
        _remoteAzmMove = false;
        _remoteElevMove = true;
        // set the rotor control outputs to down
        digitalWrite(_G5500UpPin, LOW);
        digitalWrite(_G5500DownPin, HIGH);
        digitalWrite(_G5500LeftPin, LOW);
        digitalWrite(_G5500RightPin, LOW);
        break;
      }

    case 'l':
    case 'L':
      {
        _gs232WActice = false;
        _elevationMove = false;
        _azimuthMove = false;
        _manualMove = true;
        _remoteAzmMove = true;
        _remoteElevMove = false;
        // set the rotor control outputs to left
        digitalWrite(_G5500UpPin, LOW);
        digitalWrite(_G5500DownPin, LOW);
        digitalWrite(_G5500LeftPin, HIGH);
        digitalWrite(_G5500RightPin, LOW);
        break;
      }

    case 'r':
    case 'R':
      {
        _gs232WActice = false;
        _elevationMove = false;
        _azimuthMove = false;
        _manualMove = true;
        _remoteAzmMove = true;
        _remoteElevMove = false;
        // set the rotor control outputs to right
        digitalWrite(_G5500UpPin, LOW);
        digitalWrite(_G5500DownPin, LOW);
        digitalWrite(_G5500LeftPin, LOW);
        digitalWrite(_G5500RightPin, HIGH);
        break;
      }

    case 's':
    case 'S':
      {
        _gs232WActice = false;
        _elevationMove = false;
        _azimuthMove = false;
        _manualMove = false;
        _remoteAzmMove = false;
        _remoteElevMove = false;
        break;
      }

    case '!':
      {
        DEBUG_FLAG = !DEBUG_FLAG;
        break;
      }

    case '>':
      {
        readPersistent();
        break;
      }

    case '<':
      {
        readCalFromSerial(incomm);
        break;
      }

    default:
      {
        // ignore everything else
        break;
      }
  }
  character = 0x00;
}

//
// 7 Segment Display
//
void displayAzEl()
{
  //Useful for calibrating and debug
  if (DEBUG_FLAG)
  {
    Serial.print("AZ (raw/val): ");
    Serial.print(_AZsensorValue);
    Serial.print(" / ");
    Serial.println(_rotorAzimuth);
    Serial.print("EL (raw/val): ");
    Serial.print(_ELsensorValue);
    Serial.print(" / ");
    Serial.println(_rotorElevation);
  }

  //Display 7 Segment TM1650
  char line[4];
  if (_lockMode)
  {
    d.displayString("Hold");
  }
  else
  {
    if (_manualMove)
    {
      //manual mode
      if (button == 0x67 || button == 0x6F)
      {
        //Left or Right (AZM)
        sprintf(line, "A%03d", (int)(_rotorAzimuth / 100.0f));
      }
      if (button == 0x47 || button == 0x4F)
      {
        //Up or Down (ELEV)
        sprintf(line, "E%03d", (int)(_rotorElevation / 100.0f));
      }
      d.displayString(line);
    }
    else
    {
      //normal mode
      if (displayCycle)
      {
        sprintf(line, "E%03d", (int)(_rotorElevation / 100.0f));
      }
      else
      {
        sprintf(line, "A%03d", (int)(_rotorAzimuth / 100.0f));
      }
      d.displayString(line);
      displayCycle = !displayCycle;
    }
  }
}

//
// TM1650 Buttons
//
void CheckButtons()
{
  button = d.getButtons();
  switch (button)
  {
    case 0x77:
      {
        //Button PWR is to enter LOCK mode
        _gs232WActice = false;
        _azimuthMove = false;
        _elevationMove = false;
        _manualMove = false;
        _remoteAzmMove = false;
        _remoteElevMove = false;
        _lockMode = !_lockMode; // toggle last status

        // set all the rotor control outputs low
        digitalWrite(_G5500UpPin, LOW);
        digitalWrite(_G5500DownPin, LOW);
        digitalWrite(_G5500LeftPin, LOW);
        digitalWrite(_G5500RightPin, LOW);

        break;
      }

    case 0x57:
      {
        if (!_lockMode)
        {
          //Button OK is to force STOP
          _gs232WActice = false;
          _azimuthMove = false;
          _elevationMove = false;
          _manualMove = false;
          _remoteAzmMove = false;
          _remoteElevMove = false;

          // set all the rotor control outputs low
          digitalWrite(_G5500UpPin, LOW);
          digitalWrite(_G5500DownPin, LOW);
          digitalWrite(_G5500LeftPin, LOW);
          digitalWrite(_G5500RightPin, LOW);
          d.displayString("StOP");
        }
        break;
      }

    case 0x67:
      {
        // Button Left
        if (!_lockMode)
        {
          _gs232WActice = false;
          _azimuthMove = false;
          _elevationMove = false;
          _manualMove = true;

          // set the rotor control outputs to left
          digitalWrite(_G5500UpPin, LOW);
          digitalWrite(_G5500DownPin, LOW);
          digitalWrite(_G5500LeftPin, HIGH);
          digitalWrite(_G5500RightPin, LOW);
        }
        break;
      }

    case 0x6F:
      {
        // Button Right
        if (!_lockMode)
        {
          _gs232WActice = false;
          _azimuthMove = false;
          _elevationMove = false;
          _manualMove = true;

          // set the rotor control outputs to right
          digitalWrite(_G5500UpPin, LOW);
          digitalWrite(_G5500DownPin, LOW);
          digitalWrite(_G5500LeftPin, LOW);
          digitalWrite(_G5500RightPin, HIGH);
        }
        break;
      }

    case 0x47:
      {
        // Button Down
        if (!_lockMode)
        {
          _gs232WActice = false;
          _azimuthMove = false;
          _elevationMove = false;
          _manualMove = true;

          // set the rotor control outputs to down
          digitalWrite(_G5500UpPin, LOW);
          digitalWrite(_G5500DownPin, HIGH);
          digitalWrite(_G5500LeftPin, LOW);
          digitalWrite(_G5500RightPin, LOW);
        }
        break;
      }

    case 0x4F:
      {
        // Button Up
        if (!_lockMode)
        {
          _gs232WActice = false;
          _azimuthMove = false;
          _elevationMove = false;
          _manualMove = true;

          // set the rotor control outputs to Up
          digitalWrite(_G5500UpPin, HIGH);
          digitalWrite(_G5500DownPin, LOW);
          digitalWrite(_G5500LeftPin, LOW);
          digitalWrite(_G5500RightPin, LOW);
        }
        break;
      }

    default:
      {
        //No key pressed, so we can assume Manual Mode is OFF
        _manualMove = false;
        break;
      }
  }
}

//
// Read Persistent Config Data from EEPROM
//
void readPersistent()
{
  int addr = 0;
  //Check if eeprom has never been written (255 value)
  byte prevWritten = EEPROM.read(addr);
  addr++;
  Serial.println(">");
  Serial.println(prevWritten);
  if (prevWritten == 37)
  {
    //Old Data exists, continue reading
    EEPROM.get(addr, _azAdZeroOffset);
    Serial.println(_azAdZeroOffset);
    addr += sizeof(long);
    EEPROM.get(addr, _elAdZeroOffset);
    Serial.println(_elAdZeroOffset);
    addr += sizeof(long);
    EEPROM.get(addr, _azScaleFactor);
    Serial.println(_azScaleFactor);
    addr += sizeof(long);
    EEPROM.get(addr, _elScaleFactor);
    Serial.println(_elScaleFactor);
    addr += sizeof(long);
    EEPROM.get(addr, _closeEnough);
    Serial.println(_closeEnough);
    addr += sizeof(long);
    EEPROM.get(addr, _maxRotorAzimuth);
    Serial.println(_maxRotorAzimuth);
    addr += sizeof(long);
    EEPROM.get(addr, _maxRotorElevation);
    Serial.println(_maxRotorElevation);
    addr += sizeof(long);
  }
  else
  {
    //No old data exist, save defaults
    savePersistent();
  }
}

//Save Persistent Config Data
void savePersistent()
{
  int addr = 0;
  //Write indicator of config in memory (37 value) a random value different of 255
  EEPROM.update(addr, 37);
  addr++;
  //Start writing of values
  EEPROM.put(addr, _azAdZeroOffset);
  addr += sizeof(long);
  EEPROM.put(addr, _elAdZeroOffset);
  addr += sizeof(long);
  EEPROM.put(addr, _azScaleFactor);
  addr += sizeof(long);
  EEPROM.put(addr, _elScaleFactor);
  addr += sizeof(long);
  EEPROM.put(addr, _closeEnough);
  addr += sizeof(long);
  EEPROM.put(addr, _maxRotorAzimuth);
  addr += sizeof(long);
  EEPROM.put(addr, _maxRotorElevation);
  addr += sizeof(long);
}

//Read Calibration Data from Serial Stream
void readCalFromSerial(String calValStr)
{
  //String calValStr = Serial.readStringUntil('\n');
  //Serial.flush();
  Serial.println(calValStr);
  long a, b, c, d, e, f, g;
  a = _azAdZeroOffset;
  b = _elAdZeroOffset;
  c = _azScaleFactor;
  d = _elScaleFactor;
  e = _closeEnough;
  f = _maxRotorAzimuth;
  g = _maxRotorElevation;
  if (sscanf(calValStr.c_str(), "<%ld_%ld_%ld_%ld_%ld_%ld_%ld", &a, &b, &c, &d, &e, &f, &g) == 7)
  {
    //apply
    _azAdZeroOffset = a;
    _elAdZeroOffset = b;
    _azScaleFactor = c;
    _elScaleFactor = d;
    _closeEnough = e;
    _maxRotorAzimuth = f;
    _maxRotorElevation = g;
    //Save in EEPROM
    savePersistent();
  }

}
