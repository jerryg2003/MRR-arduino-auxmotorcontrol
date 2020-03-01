// DC Motor Control with Speed and Rotation
// Using H-Bridge
// from howtomechatronics.com

//***NOTE: Only completed for Motor A, although variables are included for Motor B
// Sketch by Jerry Grochow (c)
// 2018-10-22
// 2018-12-27
// 2019-01-09
// 2019-04-12 V4
// 2019-06-29 V4b starting to change CMRI calls and variables to bool
// 2019-08-29 v5 replace millis() with curTime

//CMRI output bit 47=1 when JMRI panel active; allow motor on-off switch even when not
//CMRI input bit 23 handshakes back to JMRI
//Set up Light in JMRI controlled by CMRI bit 47
//Set up Sensor in JMRI controlled by CMRI bit 23

//CMRI output bits 0 (on, off), 1-2 (speed up-down), [[3 (rotation direction)]] control Motor A
//CMRI input bits 0,1,2,3,4,5,6,7 indicate speed, 8 direction, 9 sensor
//CMRI output bits 10,11,12,13 control Motor B
//CMRI input bits 10,11,12,13,14,15,16,17 indicate speed, 18 direction, 19 sensor

//Set up Lights in JMRI to control Motors on/off, speed, and direction
//  MotorA: CMRI output bit 0 (ON,OFF), bit 1(Increase speed), bit 2(decrease speed), [[bit 3 (change direction)]]
//  MotorB: CMRI output bit 10 (ON,OFF), bit 11(Increase speed), bit 12(decrease speed), bit 13 (change direction)
//Set up Sensors in JMRI to indicate Motors speed and direction action taken and sensor
//  MotorA: CMRI input bit 0-7 speed, 8 direction,9 sensor
//  MotorB: CMRI input bit 10-17 speed, 18 direction, 19 sensor

//Explanation of STATE CHANGE TABLE at end of this file

//Manual switch setting of ON will override JMRI attempt to turn off motor
//Manual switch setting of OFF after ON will override JMRI.
//Can only change speed and direction when JMRI active.

//Sensor Active will only turn on motor when JMRI active only

//Saves speed and direction in EEPROM


#include <EEPROM.h>
#include <Auto485.h>
#include <CMRI.h>

const int CMRI_ADDR = 11;             //CMRI Board #11
const int DE_PIN = 2;                 //For RS485

//Set up arduino pin assignments
const int enA = 3;                    //Must be PWM pin
const int pin4 = 4;                   //NOT USED
const int in1 = 5;
const int in2 = 6;
const int in3 = 7;
const int in4 = 8;
const int enB = 9;                    //Must be PWM pin
const int switchA = 10;               //**Pin 11 = LOW
const int switchB = 14;               //**Pin 15 = LOW
const int sensorA = 17;               //**Pin 18 = GND, Pin 19 = 5V

const int lowestOutputPin = 3;
const int highestOutputPin = 9;

//Other constants: speed, sensor, times
const bool switchON = LOW;            //Switch state ON is LOW
const bool switchOFF = HIGH;
const bool sensorActive = HIGH;        //Sensor active state is LOW for REFLECTED BEAM, HIGH for BROKEN BEAM
const bool sensorInactive = LOW;

const int startSpeedA = 40;           //Motor speed constraints
const int startSpeedB = 40;
const int maxSpeedA = 200;
const int maxSpeedB = 200;

const int actionDelay = 1000;         //Time to wait between actions
const int speedChangeDelay = 5000;
const int sensorDelay = 2000;
const int waitTime = 12;

//Working variables
int speedA = 0;
int speedB = 0;
int speedChangeA = 0;                 //Request via JMRI
int speedChangeB = 0;                 //Request via JMRI
int rotDirectionA = 1;                //Request via JMRI
int rotDirectionB = 0;                //Request via JMRI
//int rotDirChangeA = 0;
int rotDirChangeB = 0;
int up = 0;
int down = 0;
int dir = 0;

bool motorAON = false;                //Actual state of motor
bool motorBON = false;

bool prevJMRIMotorStateA = false;          //Initialize JMRI commanded state of each motor
bool prevJMRIMotorStateB = false;
bool curJMRIMotorStateA  = false;
bool curJMRIMotorStateB  = false;

int prevSwitchStateA     = switchOFF;     //Initialize state of each switch and sensor
int prevSwitchStateB     = switchOFF;
int prevSensorStateA     = sensorInactive;
int curSwitchStateA      = switchOFF;
int curSwitchStateB      = switchOFF;
int curSensorStateA      = sensorInactive;

bool stateChangeA = false;             //Determined by this sketch
bool stateChangeB = false;


bool prevJMRIAvail = false;            //State of JMRI Panel
bool curJMRIAvail = false;
int loopCount = 0;

unsigned long int actionDelayTime = 0; //Set up to lock out actions for some time
unsigned long int speedChangeDelayTime  = 0;
unsigned long int sensorDelayTime = 0;
unsigned long int curTime = 0;
int speedIncrement = 0;


Auto485 bus(DE_PIN);

CMRI cmri(CMRI_ADDR, 24, 48, bus); //SMINI = 24 inputs, 48 outputs


//========================================================================================

void setup() {

  //Define EXTRA arduino pins used for convenience by my system
  pinMode(sensorA + 1, OUTPUT);   digitalWrite (sensorA + 1, LOW);   //Need GND for sensor
  pinMode(sensorA + 2, OUTPUT);   digitalWrite (sensorA + 2, HIGH);  //Need 5V for sensor
  pinMode(switchA + 1, OUTPUT);   digitalWrite (switchA + 1, LOW);   //Use following pin for ground for each switch
  pinMode(switchB + 1, OUTPUT);   digitalWrite (switchB + 1, LOW);

  //Define pins for this sketch
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = lowestOutputPin; i < (highestOutputPin + 1); i++) {
    pinMode(i, OUTPUT);
  }
  pinMode(switchA, INPUT_PULLUP);                 //Input setup
  pinMode(switchB, INPUT_PULLUP);
  pinMode(sensorA, INPUT_PULLUP);

  //Get saved speed and direction, if any
  speedA = constrain(EEPROM.read(0), startSpeedA, maxSpeedA);
  // rotDirectionA = EEPROM.read(1);              //FUTURE USE
  // speedB = constrain(EEPROM.read(2), startSpeedB, maxSpeedB);
  // rotDirectionB = EEPROM.read(3);

  // Set initial rotation direction
  if (rotDirectionA == 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite (enA, 0);
    motorAON = false;
    cmri.set_bit(8, false);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite (enA, 0);
    motorAON = false;
    cmri.set_bit(8, true);
  }
  //  if (rotDirectionB == 0) {                   //FUTURE USE
  //    digitalWrite(in3, HIGH);
  //    digitalWrite(in4, LOW);
  //    analogWrite (enB, 0);
  //    motorBON = false;
  //  }
  //  else {
  //    digitalWrite (in3, LOW);
  //    digitalWrite (in4, HIGH);
  //    analogWrite (enB, 0);
  //    motorBON = false;
  //  }


  //*****************NAME SKETCH*******************************************
  //                                                                      *
  bus.begin (19200);                                                   // *
  bus.println ("Setup complete: CMRI485_2MOTOR-v5 2019-08-29");   // *
  bus.println (String(EEPROM.read(0)) + " " + String(EEPROM.read(1)) + " " + String(EEPROM.read(2)) + " " + String(EEPROM.read(3)));
  //                                                                      *
  //***********************************************************************

  //Flash LED that board is ready
  digitalWrite (LED_BUILTIN, HIGH);
  delay (1000);
  digitalWrite (LED_BUILTIN, LOW);
  delay(200);
  digitalWrite (LED_BUILTIN, HIGH);
  delay (300);
  digitalWrite (LED_BUILTIN, LOW);
  delay(100);
  digitalWrite (LED_BUILTIN, HIGH);
  delay (300);
  digitalWrite (LED_BUILTIN, LOW);

  actionDelayTime = millis() - 1;                    //Set action times to now!
  speedChangeDelayTime = actionDelayTime;
  sensorDelayTime = actionDelayTime;
  speedIncrement = (maxSpeedA - startSpeedA) / 8.0;  //For JMRI panel speed display

}


//=======================================================================================

void loop() {

  curTime = millis();

  //************  SEE IF CMRI NODE TURNED ON FROM JMRI PANEL ******************************************************
  cmri.process();
  if (cmri.get_bit(47)) {                             //CMRI node is turned ON in JMRI panel
    if (!prevJMRIAvail) { 
      cmri.set_bit(23, true);                         //Feedback to JMRI
      curJMRIAvail = true;
      //Set speed display bits for JMRI panel
      int speedDisplay = constrain(((speedA - startSpeedA) + 1) / speedIncrement, 0, 7);
      for (int i = 0; i < 8; i++ ) {
        if (i < speedDisplay + 1) cmri.set_bit(i, true); //Turn on speed display bits in JMRI
        else cmri.set_bit(i, false);
      }
    }
  }
  else {                                              //CMRI node is turned OFF from JMRI panel
    curJMRIAvail = false;
    curJMRIMotorStateA = false;                       //Set motors state off from JMRI
    curJMRIMotorStateB = false;
    curSensorStateA = sensorInactive;                 //Set sensor inactive
    cmri.set_bit(23, false);                          //Feedback to JMRI
  }

  //*********  Get action commands from switch, sensor, and JMRI  *****************************************
  curSwitchStateA = digitalRead(switchA);            //Get state of switch from Arduino (LOW means ON)
  curSwitchStateB = digitalRead(switchB);

  //Get motor state command (from JMRI) and sensor (from Arduino) if JMRI available
  if (curJMRIAvail) {                                //Only read if JMRI panel is available
    curJMRIMotorStateA = cmri.get_bit(0);            //Get state of Motor as known to JMRI (1 means ON)
    curJMRIMotorStateB = cmri.get_bit(10);
    curSensorStateA = digitalRead (sensorA);         //Get sensor state
    if (curSensorStateA == sensorActive) {           //If sensor active, simiulate JMRI command to turn motor on
      curJMRIMotorStateA = true;                     //NOTE: will NOT change indicator on JMRI panel
      cmri.set_bit(9, true);                         //Set occupancy sensor in JMRI
    }
    else {
      if (sensorDelayTime <= curTime)  cmri.set_bit(9, false); //Set sensor off in JMRI after time delay
    }
  }
  else {                                             //JMRI panel is NOT available
    if (prevJMRIAvail) {                             //Just changed to not available
      prevSwitchStateA = switchON;                   //Simulate switch previously being on so that motor will be shut off if switch not on now
    }
  }

  //*****************  Process speed and direction change request if JMRI available *******************************
  if (curJMRIAvail) {                                //These capabilities only available through JMRI
    speedChangeA = ProcessSpeedChangeA();
  }

  //*******************  Process state change  ********************************************************************
  stateChangeA = ProcessStateChangeA();

  //**************  Save state for next go round  *****************************************************************
  prevJMRIAvail = curJMRIAvail;
  prevJMRIMotorStateA = curJMRIMotorStateA;
  prevSwitchStateA = curSwitchStateA;
  prevSensorStateA = curSensorStateA;
  EEPROM.write(0, speedA);
  EEPROM.write(1, rotDirectionA);

  //Set processing delays based on different actions
  if (stateChangeA or stateChangeB) {
    actionDelayTime = curTime + actionDelay;                    //Set time space between actions
  }
  if (speedChangeA != 0 or speedChangeB != 0) {
    speedChangeDelayTime = curTime + speedChangeDelay;
  }
  if (curSensorStateA == sensorActive) {
    sensorDelayTime = curTime + sensorDelay;
  }
}
//                                                                                                                    **
//                                                                                                                    **
//************************* END OF LOOP  *******************************************************************************


//************************** FUNCTIONS    ******************************************************************************
//**********************************************************************************************************************
//**********************************************************************************************************************

//***************** SPEED CHANGE PROCESS FOR MOTOR A ************************************************
int ProcessSpeedChangeA () {

  int net = 0;
  if (curJMRIMotorStateA or curSwitchStateA == switchON) {
    if (speedChangeDelayTime < curTime)  {          //Wait for a while when speed changed
      up = cmri.get_bit(1);
      down =  cmri.get_bit(2);
      //dir = cmri.get_bit(3);
      net = up - down;                      // See if speed change pending (+ and - cancel)
      speedA = constrain (speedA + (net * speedIncrement), startSpeedA, maxSpeedA);  //Allow speed change within limits

      //Set speed display bits for JMRI panel
      int speedDisplay = constrain(((speedA - startSpeedA) + 1) / speedIncrement, 0, 7);
      for (int i = 0; i < 8; i++ ) {
        if (i < speedDisplay + 1) cmri.set_bit(i, true); //Turn on speed bits from JMRI
        else cmri.set_bit(i, false);
      }
    }
  }
  return (net);
}
//********************** END SPEED CHANGE PROCESS FOR MOTOR A *****************************************

//**********  STATE CHANGE PROCESS FOR MOTOR A  *******************************************************
bool ProcessStateChangeA () {

  if (curSwitchStateA == switchON and prevSwitchStateA == switchOFF) {         //Switch turn on: turn on motor
    analogWrite (enA, speedA + 10);      //Give it a boost on startup
    motorAON = true;
    delay (3 * waitTime);
    analogWrite (enA, speedA);
    delay (waitTime);
    return (true);
  }

  if (curSwitchStateA == switchOFF and prevSwitchStateA == switchON) {       //Switch turned off: turn off motor
    analogWrite (enA, 0);
    motorAON = false;
    delay (waitTime);
    return (true);
  }

  //If gets to here, swwitch stays ON or OFF; allow JMRI actions
  if (curJMRIAvail == true) {                                      //Switch stays off; JMRI controls
    if (actionDelayTime > curTime) return (false);                //Not enough time has elapsed
    if (sensorDelayTime > curTime) return (false);
    cmri.set_bit(16, false);   cmri.set_bit(17, false);   cmri.set_bit(18, false);      cmri.set_bit(19, false);   //DEBUG

    if (!curJMRIMotorStateA and prevJMRIMotorStateA) {    //JMRI turns off; turn off motor
      analogWrite (enA, 0);
      motorAON = false;
      delay (waitTime);
      cmri.set_bit(16, true);                    //DEBUG
      return (true);
    }

    if (curJMRIMotorStateA)  {                              //JMRI signals to turn on motor
      if (!prevJMRIMotorStateA) {                           //Turn on motor
        analogWrite (enA, speedA + 10);                           //Give it a boost on startup
        motorAON = true;
        delay (3 * waitTime);
        analogWrite (enA, speedA);
        delay (waitTime);
        cmri.set_bit(17, true);                   //DEBUG
        return (true);
      }
      if (speedChangeA != 0) {                                    //If motor on and speedchange...
        analogWrite (enA, speedA);
        motorAON = true;
        delay (waitTime);
        cmri.set_bit(18, true);                    //DEBUG
        return (true);
      }
      return (false);                                             //No state change: JMRI command ON
    }

    //If gets to here, time delay past, JMRI panel available, JMRI command OFF
    if (curSensorStateA == sensorInactive and curSwitchStateA == switchOFF and motorAON == true) {
      analogWrite (enA, 0);
      motorAON = false;
      delay (waitTime);
      cmri.set_bit(19, true);                      //DEBUG
      return (true);
    }
  }
  return (false);                                                 //No state change
  //**************  END STATE CHANGE PROCESS FOR MOTOR A  ***********************************
}

//***********************************************************************************************************************************
//State table for motor control actions                                                                                            **
// curJMRIMotorState   curSwitchState   prevJMRIMotorState  prevSwitchState                                                        **

//  if (curSwitchStateA == switchON and prevSwitchStateA == switchOFF)
//      0                ON               0                OFF          --> Switch turned on,                            TURN ON MOTOR
//      0                ON               1                OFF          --> JMRI turned off motor; switch overrides;     TURN ON MOTOR
//      1                ON               0                OFF          --> JMRI turned onmotor; switch turned on motor; TURN ON MOTOR
//      1                ON               1                OFF          --> Motor on; switch turned on;                  NO ACTION NECESSARY

//  if (curSwitchStateA == switchOFF and prevSwitchStateA == switchON)
//      0                OFF               0                ON          --> Switch turned off;                           turn OFF motor
//      0                OFF               1                ON          --> JMRI and switch turned off motor;            turn OFF motor
//      1                OFF               0                ON          --> JMRI turned on motor; switch overridese;     turn OFF motor
//      1                OFF               1                ON          --> Switch turned off while motor on;            turn OFF motor

//  if (curJMRIMotorStateA == 0 and prevJMRIMotorStateA == 1)
//      0                OFF               1                OFF          --> JMRI turned off motor;                       turn OFF motor
//  if (curJMRIMotorStateA == 1 and prevJMRIMotorStateA == 0)
//      1                OFF               0                OFF          --> JMRI turned on motor;                        TURN ON MOTOR
//      0                OFF               0                OFF          --> Motor OFF;                                   No action
//      1                OFF               1                OFF          --> Motor on;                                    No action

//      0                ON                0                ON           --> Switch continues on;                         No action
//      0                ON                1                ON           --> Motor already on via switch; overrides JMRI  No action
//      1                ON                0                ON           --> Motor on;                                    No action
//      1                ON                1                ON           --> Motor ON;                                    No action

//                                                                                                                                 **
//***** Sensor active when JMRI available sets switch to ON                                                                        **
//***********************************************************************************************************************************
