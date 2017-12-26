/*
Step Indexer v. 3.1

This program controls the position of a stepper output shaft via
manual input and displays the result.  It also provides temperature
information about the stepper motor and its driver board.

The hardware assumes a bipolar stepper motor with a TB6065 driver board
and the Sainsmart LCD/keypad sheild with two TMP36 temp sensors

2.0 Created  March 2013 by Gary Liming
2.1 Frozen   October 2013 by Gary Liming
2.2 Frozen   February 2014 by Gary Liming
2.3 Frozen   September 2014 by Gary Liming
3.0 Frozen   March 2017 by Tomasz P. Musielak
3.1 Frozen   December 2017 by Tomasz P. Musielak
*/

//  First, include some files for the LCD display, and its motor
#include <Arduino.h>
#include <TimerOne.h>

#ifndef I2C_LCD
#include <LiquidCrystal.h>
#endif // !I2C_LCD

#ifdef I2C_LCD
#include <LiquidCrystal_I2C.h>
#endif // DEBUG

#define DEBUG
#pragma region Motor parameters

//  Next define your parameters about your motor and gearing

#define StepsPerRevolution 20000  // Change this to represent the number of steps
//   it takes the motor to do one full revolution
//   200 is the result of a 1.8 degree per step motor
#define Microsteps 1            // Depending on your stepper driver, it may support
//   microstepping.  Set this number to the number of microsteps
//   that is set on the driver board.  For large ratios, you may want to 
//   use no microstepping and set this to 1.

#define GearRatioCount 3          // Number of gear ratios defined

#define AngleIncrement 5        // Set how much a keypress changes the angle setting

#define CW LOW                 // Define direction of rotation
#define CCW HIGH               // If rotation needs to be reversed, swap HIGH and LOW here

#define MotorISRPeriod 15  // Setting below 15 freezes the program when the motor is running.
#define DefaultMotorSpeed MotorISRPeriod  

#pragma endregion Motor parameters

#define version "3.1"

#pragma region Menu items
// define menu screen ids
#define mainmenu  0
#define divisions 1
#define anglemode 2
#define runmode   3
#define jogmode   4
#define ratiomode 5
#define tempmode  6
#define numModes  6     // number of above menu items to choose, not counting main menu

#define moveangle 10
#define movesteps 11

#pragma endregion Menu items

#define Temperature_C 1
#define Temperature_F 2

// Define the pins that the driver is attached to.
// Once set, this are normally not changed since they correspond to the wiring.

#define motorSTEPpin   A5   // output signal to step the motor
#define motorDIRpin    A4   // output siagnal to set direction
#define motorENABLEpin A3   // output pin to power up the motor
#define AnalogKeyPin   0    // keypad uses A0
#define SinkTempPin    1    // temp sensor for heatsink is pin A1
#define MotorTempPin   2    // temp sensor for motor is pin A2

#define pulsewidth     2    // length of time for one step pulse
#define ScreenPause    800  // pause after screen completion
#define ADSettleTime   10   // ms delay to let AD converter "settle" i.e., discharge

#define JogStepsIncrement 100    // initial value of number of steps per keypress in test mode
#define TSampleSize 7          // size of temperature sampling to average

#define KeyDebounceDelayMs 100
#define KeyRepeatDelayMs 333

// create lcd display construct

#ifndef I2C_LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  //Data Structure and Pin Numbers for the LCD/Keypad
#endif // !I2C_LCD

#ifdef I2C_LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#endif // I2C_LCD

// define LCD/Keypad buttons

#define NO_KEY 0
#define SELECT_KEY 1
#define LEFT_KEY 2
#define UP_KEY 3
#define DOWN_KEY 4
#define RIGHT_KEY 5

#define CW_SYMBOL byte(1)
#define CCW_SYMBOL byte(2)
#define DEG_C_SYMBOL byte(3)
#define DEG_F_SYMBOL byte(4)

#ifdef DEBUG
unsigned long last_debug_displayed = 0;
#endif // DEBUG

// create global variables

int    cur_mode = mainmenu;
int    mode_select = divisions;
int    current_menu;
float  cur_angle = 0;
int    cur_key;
int    num_divisions = 1;
int    numjogsteps = JogStepsIncrement;
unsigned long stepsperdiv;
int    cur_pos = 0;
int    cur_dir = CW;
volatile unsigned int motorSpeedDelay = DefaultMotorSpeed;

unsigned long motorSteps; // Total number of motor steps per revolution.

// Change these values to reflect any front end gearing you 
//   are using for three different devices.  If no gearing, then
//   define this value to be 1.  GearRatio1 is the default
//   this is the bottom of the ratio - usually 1, as in 3 to 1, but you may change that here.
unsigned long gear_ratio_top_array[GearRatioCount]    = { 15, 3, 1 };
unsigned long gear_ratio_bottom_array[GearRatioCount] = {  1, 1, 1 };

int    gearratioindex = 0;      // the first array element starts with 0 and is the default ration chosen

byte temperatureMode = Temperature_C;
byte temperatureSymbol = DEG_C_SYMBOL;

bool drawScreenNeeded = true;

bool modeEnabled[numModes] = { true, true, true, true, true, false /* Temp mode disabled */ };

volatile int dirToGo = 0;
volatile unsigned long stepsToGo = 0;
volatile bool motorEnabled = false;
volatile unsigned long stepsExecuted = 0;

unsigned long cycles = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
    
  //  Create some custom characters for the lcd display
  byte c_CW[8] = { 0b01101,0b10011,0b10111,0b10000,0b10000,0b10000,0b10001,0b01110 }; // Clockwise
  byte c_CCW[8] = { 0b10110,0b11001,0b11101,0b00001,0b00001,0b00001,0b10001,0b01110 }; // CounterClockWise

  byte c_DegreeF[8] = { 0b01000,0b10100,0b01000,0b00111,0b00100,0b00110,0b00100,0b00100 }; //  degreeF
  byte c_DegreeC[8] = { 0b01000,0b10100,0b01011,0b00101,0b00100,0b00100,0b00101,0b00011 }; //  degreeC
    
#ifdef I2C_LCD
  lcd.init();
  lcd.clear();
  lcd.backlight();
#endif // I2C_LCD

  lcd.createChar(CW_SYMBOL, c_CW);
  lcd.createChar(CCW_SYMBOL, c_CCW);
  lcd.createChar(DEG_C_SYMBOL, c_DegreeC);
  lcd.createChar(DEG_F_SYMBOL, c_DegreeF);
  
  // begin program

  motorSteps = (gear_ratio_top_array[gearratioindex] * Microsteps * StepsPerRevolution) / gear_ratio_bottom_array[gearratioindex];
  
  printIntroScreen();
  delay(1500);                        // wait a few secs
  lcd.clear();
  
  pinMode(motorSTEPpin, OUTPUT);     // set pin 3 to output
  pinMode(motorDIRpin, OUTPUT);      // set pin 2 to output
  pinMode(motorENABLEpin, OUTPUT);   // set pin 11 to output

  digitalWrite(motorSTEPpin, LOW);
  digitalWrite(motorDIRpin, LOW);
  digitalWrite(motorENABLEpin, HIGH);
  
  Timer1.initialize(MotorISRPeriod);
  Timer1.stop();
  Timer1.attachInterrupt(actionMotor);

  displayMainMenu();
}                                    // end of setup function



void loop()                          // main loop services keystrokes
{ 
  int key = getKey();
  runModes(key);
  
  if (key != NO_KEY)
  {
    displayScreen(cur_mode);
  }

#ifdef DEBUG
  unsigned long now = millis();
  if (now - last_debug_displayed > 2000)
  {
    printDebug();
  }
#endif // DEBUG
  cycles++;
}  

void runModes(int key)
{
  switch (cur_mode)                    // execute the keystroke
  {
  case mainmenu:                    // main menu
    doMainMenu(key);
    break;
  case tempmode:                    // call temps
    doTempMode(key);    
    break;
  case divisions:                    // call steps
    doDivisionsMode(key);    
    break;
  case anglemode:                   // call angles
    doAngleMode(key);    
    break;
  case runmode:                     // call run  
    doRunMode(key);    
    break;
  case jogmode:                    // call jog
    doJogMode(key);    
    break;
  case ratiomode:                 // call ratio
    doRatioMode(key);    
    break;
  }
}

void printDebug()
{
  unsigned long now = millis();  
    
  Serial.print(now);
  Serial.print(" cycles: ");
  Serial.print(cycles);
  Serial.print(" mode: ");
  Serial.print(mode_select);
  Serial.print(" enabled: ");
  Serial.print(motorEnabled);
  Serial.print(" stepsTogo: ");
  Serial.print(stepsToGo);
  Serial.print(" dir: ");
  Serial.print(dirToGo);
  Serial.print(" stepsExecuted: ");
  Serial.println(stepsExecuted);

//  String s = "Last r: [";
//  for(int i = 0; i < r_readings_count; i++)
//  {
//    s.concat(last_r_readings[i]);
//    s.concat(",");    
//  }    
//  s.concat("]");
//  Serial.println(s);
//  r_readings_count = 0;  
  last_debug_displayed = now;
  cycles = 0;
}

void doMainMenu(int key)
{ 
  switch (key)
  {
  case LEFT_KEY:
  case UP_KEY:
    mode_select++;    
    if(!modeEnabled[mode_select - 1])
      mode_select++;
    if (mode_select > numModes)  // wrap around menu
      mode_select = 1;          
    break;
  case RIGHT_KEY:
  case DOWN_KEY:
    mode_select--;    
    if (mode_select < 1)         // wrap around menu
      mode_select = numModes;      
    if(!modeEnabled[mode_select - 1])
      mode_select--;
    break;
  case SELECT_KEY:         // user has picked a menu item
    setMode(mode_select);      
    break;
  }
}

void setMode(int mode)
{
  Serial.print("Entering mode ");
  Serial.println(mode);
  displayScreen(mode);  
  cur_mode = mode;
}

void doDivisionsMode
(int key)
{
  switch (key)
  {
  case UP_KEY:
    num_divisions++;      
    cur_pos = 0;
    break;
  case DOWN_KEY:
    if (num_divisions > 1)
      num_divisions--;      
    cur_pos = 0;
    break;
  case LEFT_KEY:
    cur_dir = CCW;
    stepsperdiv = (motorSteps / num_divisions);
    moveMotor(stepsperdiv, cur_dir);
    
    cur_pos--;
    if (cur_pos < 0)
      cur_pos = num_divisions - 1;      
    break;
  case RIGHT_KEY:
    cur_dir = CW;
    stepsperdiv = (motorSteps / num_divisions);
    moveMotor(stepsperdiv, cur_dir);
    
    cur_pos++;
    if (cur_pos == num_divisions)
      cur_pos = 0;      
    break;
  case SELECT_KEY:
    cur_pos = 0;       // reset position
    num_divisions = 1; // reset number of divisions
    stepsToGo = 0;
    disableMotor();
    returnToMenu();
    return;
  }
}

void returnToMenu()
{
  setMode(mainmenu);
}

void doAngleMode(int key)
{
  switch (key)
  {
  case UP_KEY:
    if ((cur_angle + AngleIncrement) < 361)
      cur_angle += AngleIncrement;
    break;
  case DOWN_KEY:
    if ((cur_angle - AngleIncrement) > -1)
      cur_angle -= AngleIncrement;
    break;
  case LEFT_KEY:
    cur_dir = CCW;
    stepsperdiv = ((motorSteps * cur_angle) / 360);
    moveMotor(stepsperdiv, cur_dir);
    break;
  case RIGHT_KEY:
    cur_dir = CW;
    stepsperdiv = ((motorSteps * cur_angle) / 360);
    moveMotor(stepsperdiv, cur_dir);
    break;
  case SELECT_KEY:
    cur_angle = 0; // reset angle to default of zero
    disableMotor();
    returnToMenu();
    return;
    break;
  }
}

volatile bool run_enabled = false;
volatile unsigned int motorSpeedPercentage = 100;

void doRunMode(int key)
{
  switch (key)
  {
  case UP_KEY:                        // bump up the speed    
    if(motorSpeedPercentage < 100) 
    {
      motorSpeedPercentage += 10;
      motorSpeedDelay = getMotorSpeedDelay(motorSpeedPercentage);
      setMotorSpeed(motorSpeedDelay);
    }     
    break;
  case DOWN_KEY:                      // bump down the speed
    if(motorSpeedPercentage > 10) 
    {
      motorSpeedPercentage -= 10;
      motorSpeedDelay = getMotorSpeedDelay(motorSpeedPercentage);
      setMotorSpeed(motorSpeedDelay);
    }    
    break;
  case LEFT_KEY:                      // set direction
    if(cur_dir == CCW && run_enabled)
    {
      run_enabled = false;
      stopMotor();
      break;
    }
    cur_dir = CCW;          
    run_enabled = true;
    break;
  case RIGHT_KEY:                     // set other direction
    if(cur_dir == CW && run_enabled)
    {
      run_enabled = false;
      stopMotor();
      break;
    }
    cur_dir = CW;      
    run_enabled = true;
    break;
  case SELECT_KEY:                   // user wants to stop
    motorSpeedDelay = DefaultMotorSpeed;   // reset speed
    run_enabled = false;

    stopMotor();
    returnToMenu();
    return;
  }  

  if(run_enabled && (stepsToGo < 1000))
  {
    moveMotor(5000, cur_dir);  
  }
}

int getMotorSpeedDelay(unsigned int percentage)
{
  return (int)((float)DefaultMotorSpeed / ((float)percentage / 100.0f));
}

void setMotorSpeed(unsigned int motorDelay)
{  
  Timer1.stop();
  Timer1.setPeriod(motorDelay);  
  
  Timer1.restart();  
  Timer1.start();  
}

unsigned long last_displayed = 0;

void doTempMode(int key)
{ 
  if (key == SELECT_KEY)
  {
    returnToMenu();
    return;
  }    
  
  unsigned long now = millis();
  if (now - last_displayed > ScreenPause)
  {
    displayTempMode();
    last_displayed = now;
  }
}

void doJogMode(int key)
{
  switch (key)
  {
    case UP_KEY:                          // bump the number of steps
      numjogsteps += JogStepsIncrement;
      break;
    case DOWN_KEY:                        // reduce the number of steps
      if (numjogsteps > JogStepsIncrement)
        numjogsteps -= JogStepsIncrement;
      break;
    case LEFT_KEY:                        // step the motor CCW
      moveMotor(numjogsteps, CCW);
      break;
    case RIGHT_KEY:                       // step the motor CW
      moveMotor(numjogsteps, CW);
      break;
    case SELECT_KEY:                      // user want to quit
      numjogsteps = JogStepsIncrement;
      stopMotor();
      disableMotor();
      returnToMenu();
      break;
  }
}

void doRatioMode(int key)
{
    switch (key)
    {
    case UP_KEY:                          // bump the number of steps
      ++gearratioindex;
      if (gearratioindex > GearRatioCount
     - 1)  //wrap around if over the max
        gearratioindex = 0;
      motorSteps = (gear_ratio_top_array[gearratioindex] * Microsteps * StepsPerRevolution) / gear_ratio_bottom_array[gearratioindex];
      break;
    case DOWN_KEY:                        // reduce the number of steps
      --gearratioindex;
      if (gearratioindex < 0)
        gearratioindex = GearRatioCount
       - 1;    // wrap around if already at the bottom  
      motorSteps = (gear_ratio_top_array[gearratioindex] * Microsteps * StepsPerRevolution) / gear_ratio_bottom_array[gearratioindex];
      break;
    case LEFT_KEY:                        //  Left and Right keys do nothing in this mode       
    case RIGHT_KEY:
      break;
    case SELECT_KEY:                      // user want to quit
      numjogsteps = JogStepsIncrement;
      stopMotor();
      disableMotor();
      returnToMenu();
      break;
    }
}



void displayScreen(int menunum)        // screen displays are here
{
  Serial.print("Displaying screen for mode ");
  Serial.println(menunum);    
  
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (menunum)
  {
  case mainmenu:
    displayMainMenu();
    break;
  case tempmode:
    displayTempMode();
    break;
  case divisions:
    displayDivisionsMode();
    break;
  case movesteps:
    displayMoveSteps();
    break;
  case anglemode:
    displayAngleMode();
    break;
  case moveangle:
    displayMoveAngle();
    break;
  case runmode:
    displayRunMode();
    break;
  case jogmode:
    displayJogMode();
    break;
  case ratiomode:
    displayRatioMode();
    break;
  }

  return;
}

void moveMotor(unsigned long steps, int dir)
{ 
  if (stepsToGo < 1) // Not executing any moves at present.
  {
    noInterrupts();
    enableMotor();
    setDirection(dir);
    stepsToGo = steps;

    interrupts();
    
    // Start execution.
    Timer1.restart();
    Timer1.start();    
  }
  else
  {
    noInterrupts();
    if(dir == dirToGo)
    {
      stepsToGo += steps;
    }    
    else
    {
      setDirection(dir);
      stepsToGo = steps;
    }
    interrupts();
  }  
}

void stopMotor()
{
  noInterrupts();
  stepsToGo = 0;
  disableMotor();
  interrupts();
  Timer1.stop();  
}

void enableMotor()
{
  motorEnabled = true;
  digitalWrite(motorENABLEpin, LOW); // This is inverted in M542
}

void disableMotor()
{ 
  noInterrupts();  
  motorEnabled = false;
  digitalWrite(motorENABLEpin, HIGH);
  interrupts();
}

void setDirection(int dir)
{
  dirToGo = dir;
  digitalWrite(motorDIRpin, dir);
}

volatile bool motorIRSBusy = false;
volatile unsigned long lastPulse = 0;
bool nextMotorPulseState = HIGH;

// Called from timer.
void actionMotor()
{
  if (motorIRSBusy) // Avoid nesting.
  {
    return;
  }

  motorIRSBusy = true;  
 
  digitalWrite(motorSTEPpin, nextMotorPulseState);
  nextMotorPulseState = !nextMotorPulseState;  
  if (nextMotorPulseState) // Pulse complete.
  {
    --stepsToGo;
    ++stepsExecuted;
    if (stepsToGo < 1 || !motorEnabled)
    {
      stepsToGo = 0;
      Timer1.stop();
    }
  }

  motorIRSBusy = false;
}

float convertTemperature(const byte temperatureMode, const int rawTemperature)
{
  if (temperatureMode == Temperature_C)
  {
    return ((((float)(rawTemperature * 4.88758) / 10.0) - 32.0) / 1.8);
  }
  else if (temperatureMode == Temperature_F)
  {
    return ((float)(rawTemperature * 4.88758) / 10.0);
  }
  else
  {
    return NAN;
  }
}

int getTemperature(int device)
{
  float temperature = 0.0;
  int i;

  analogRead(device);                      // first time to let adc settle down
  delay(50);                               // throw first reading away

  for (i = 0; i < TSampleSize; ++i)              // take several readings
  {
    temperature += convertTemperature(temperatureMode, analogRead(device));
    delay(ADSettleTime);
  }

  return (int)(temperature / TSampleSize);   // return the average
}

volatile unsigned int lastKey = NO_KEY;


void serialEvent()
{
  byte key = NO_KEY;
  if (Serial.available() > 0)
  {    
    Serial.readBytes(&key, 1);
  }  

  switch (key)
  {
  case (byte)38:
    lastKey = UP_KEY;
    break;
  case (byte)39:
    lastKey = RIGHT_KEY;
    break;
  case (byte)40:
    lastKey = DOWN_KEY;
    break;
  case (byte)37:
    lastKey = LEFT_KEY;
    break;
  case (byte)13:
    lastKey = SELECT_KEY;
    break;
  default:
    lastKey = NO_KEY;
    break;
  }
}




unsigned long keyLastChanged = 0;
unsigned long keyLastSent = 0;

int getKey()    // routine to return a valid keystroke
{ 
  unsigned long now = millis();
  int newKey = readButton();
  if(newKey != lastKey)
  {
    keyLastChanged = now;
    keyLastSent = 0;
    lastKey = newKey;

    return NO_KEY;
  }

  if(newKey == NO_KEY)
  {
    return NO_KEY;
  }

  if(now - keyLastChanged > KeyDebounceDelayMs)
  {
    if(now - keyLastSent > KeyRepeatDelayMs) 
    {
      keyLastSent = now;
      return sendKey(newKey);
    } 
    return NO_KEY;   
  }

  return NO_KEY;
}

int sendKey(int key)
{
  Serial.print("Sending button: ");
  Serial.println(key);
  return key;
}


#define R_NO_KEY 1000
#define R_RIGHT_KEY 70
#define R_UP_KEY 150
#define R_DOWN_KEY 400
#define R_LEFT_KEY 500
#define R_SELECT_KEY 750

int readButton()
{
  analogRead(0);
  int key_in = analogRead(0);      // read ADC.

  // average values for my board were: 0, 144, 324, 505, 742
  // add approx 100 to those values to set range
  if (key_in > R_NO_KEY)      return NO_KEY;
  if (key_in < R_RIGHT_KEY)   return RIGHT_KEY;
  if (key_in < R_UP_KEY)      return UP_KEY;
  if (key_in < R_DOWN_KEY)    return DOWN_KEY;
  if (key_in < R_LEFT_KEY)    return LEFT_KEY;
  if (key_in < R_SELECT_KEY)  return SELECT_KEY;
}

#pragma region Display routines

void printIntroScreen()
{
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);                 // display flash screen
  lcd.print("Step Indexer ");
  lcd.print(version);
  lcd.setCursor(0, 1);
  lcd.print("Ratio =");
  lcd.setCursor(8, 1);
  lcd.print(gear_ratio_top_array[gearratioindex]);
  lcd.setCursor(12, 1);
  lcd.print(":");
  lcd.print(gear_ratio_bottom_array[gearratioindex]);
}

void displayMainMenu()
{
  lcd.print("Select Mode");
  lcd.setCursor(0, 1);
  lcd.print("Mode = ");
  lcd.setCursor(7, 1);
  switch (mode_select)
  {
  case(divisions):
    lcd.print("Divisions");
    break;
  case(anglemode):
    lcd.print("Angle");
    break;
  case(runmode):
    lcd.print("Run");
    break;
  case(jogmode):
    lcd.print("Jog");
    break;
  case(ratiomode):
    lcd.print("Ratio");
    break;
  case(tempmode):
    lcd.print("Temp");
    break;
  }
}

void displayDivisionsMode()
{
  lcd.print("Divisions:");
  lcd.setCursor(10, 0);
  lcd.print(num_divisions);
  lcd.setCursor(0, 1);
  lcd.print(" Position:");
  lcd.setCursor(10, 1);
  lcd.print(cur_pos);
}

void displayMoveSteps()
{
  lcd.print("Steps/Div:");
  lcd.setCursor(10, 0);
  lcd.print(stepsperdiv);
  lcd.setCursor(0, 1);
  lcd.print("Move ");
  lcd.setCursor(5, 1);
  if (cur_dir == CW)
    lcd.write(byte(1));
  else
    lcd.write(byte(2));
  lcd.setCursor(7, 1);
  lcd.print("to:");
  lcd.setCursor(10, 1);
  lcd.print(cur_pos);
}

void displayAngleMode()
{
  lcd.print("Move ");
  lcd.setCursor(5, 0);
  lcd.print((int)cur_angle);
  lcd.setCursor(8, 0);
  lcd.print(" degrees");
}

void displayMoveAngle()
{
  lcd.print("Move ");
  lcd.setCursor(5, 0);
  if (cur_dir == CW)
    lcd.write(CW_SYMBOL);
  else
    lcd.write(CCW_SYMBOL);
  lcd.setCursor(12, 0);
  lcd.print("steps");
  lcd.setCursor(0, 1);
  lcd.print("to ");
  lcd.setCursor(3, 1);
  lcd.print((int)cur_angle);
  lcd.setCursor(7, 1);
  lcd.print("degrees");
}

void displayRunMode()
{
  lcd.print("Continuous");
  if(run_enabled)
  {
    lcd.setCursor(11, 0);  
    if (cur_dir == CW)
      lcd.write(CW_SYMBOL);
    else
      lcd.write(CCW_SYMBOL);    
  }
  lcd.setCursor(0, 1);
  lcd.print("Speed = ");
  lcd.setCursor(8, 1);
  
  lcd.print(motorSpeedPercentage);
  lcd.print("%");
}

void displayJogMode()
{
  lcd.print("Jog ");
  lcd.setCursor(0, 1);
  lcd.print("Steps/jog: ");
  lcd.setCursor(11, 1);
  lcd.print(numjogsteps);
}

void displayRatioMode()
{
  lcd.setCursor(0, 0);
  lcd.print("Ratio =");
  lcd.setCursor(4, 1);
  lcd.print(gear_ratio_top_array[gearratioindex]);
  lcd.setCursor(10, 1);
  lcd.print(":");
  lcd.setCursor(11, 1);
  lcd.print(gear_ratio_bottom_array[gearratioindex]);
}

void displayTempMode()
{
  lcd.setCursor(0, 0);
  lcd.print(" Sink Temp:");
  lcd.setCursor(11, 0);
  lcd.print(getTemperature(SinkTempPin));
  lcd.setCursor(15, 0);
  lcd.write(temperatureSymbol);
  lcd.setCursor(0, 1);
  lcd.print("Motor Temp:");
  lcd.setCursor(11, 1);
  lcd.print(getTemperature(MotorTempPin));
  lcd.setCursor(15, 1);
  lcd.write(temperatureSymbol);
}

#pragma endregion