/*
Step Indexer v. 3.0

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

//  Next define your parameters about your motor and gearing

#define StepsPerRevolution 513  // Change this to represent the number of steps
//   it takes the motor to do one full revolution
//   200 is the result of a 1.8 degree per step motor
#define Microsteps 8            // Depending on your stepper driver, it may support
//   microstepping.  Set this number to the number of microsteps
//   that is set on the driver board.  For large ratios, you may want to 
//   use no microstepping and set this to 1.

// Change these three values to reflect any front end gearing you 
//   are using for three different devices.  If no gearing, then
//   define this value to be 1.  GearRatio1 is the default
//   this is the bottom of the ratio - usually 1, as in 3 to 1, but you may change that here.
#define GearRatio1top  1        
#define GearRatio1bottom 1
#define GearRatio2top  3
#define GearRatio2bottom 1
#define GearRatio3top  40
#define GearRatio3bottom 1

#define GearRatioMax 3          // number of above gear ratios defined

#define AngleIncrement 5        // Set how much a keypress changes the angle setting

#define CW HIGH                 // Define direction of rotation - 
#define CCW LOW                 // If rotation needs to be reversed, swap HIGH and LOW here

#define DefaultMotorSpeed 0     // zero here means fast, as in no delay

// define menu screen ids
#define mainmenu  0
#define stepmode  1
#define anglemode 2
#define runmode   3
#define jogmode   4
#define ratiomode 5
#define tempmode  6
#define numModes  6     // number of above menu items to choose, not counting main menu

#define moveangle 10
#define movesteps 11

#define Temperature_C 1
#define Temperature_F 2

// Define the pins that the driver is attached to.
// Once set, this are normally not changed since they correspond to the wiring.

#define motorSTEPpin   2    // output signal to step the motor
#define motorDIRpin    3    // output siagnal to set direction
#define motorENABLEpin 11   // output pin to power up the motor
#define AnalogKeyPin   0    // keypad uses A0
#define SinkTempPin    1    // temp sensor for heatsink is pin A1
#define MotorTempPin   2    // temp sensor for motor is pin A2

#define pulsewidth     2    // length of time for one step pulse
#define ScreenPause    800  // pause after screen completion
#define ADSettleTime   10   // ms delay to let AD converter "settle" i.e., discharge

#define SpeedDelayIncrement 5  // change value of motor speed steps (delay amount)
#define JogStepsIncrement 1    // initial value of number of steps per keypress in test mode
#define TSampleSize 7          // size of temperature sampling to average

#define KeyDebounceDelayMs 10
#define KeyRepeatDelayMs 500

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
int    mode_select = stepmode;
int    current_menu;
float  cur_angle = 0;
int    cur_key;
int    num_divisions = 1;
int    numjogsteps = JogStepsIncrement;
unsigned long stepsperdiv;
int    cur_pos = 0;
int    cur_dir = CW;
int    motorspeeddelay = DefaultMotorSpeed;
unsigned long motorSteps; // Total number of motor steps per revolution.

unsigned long gear_ratio_top_array[GearRatioMax] = { GearRatio1top,GearRatio2top,GearRatio3top };
unsigned long gear_ratio_bottom_array[GearRatioMax] = { GearRatio1bottom,GearRatio2bottom,GearRatio3bottom };

int    gearratioindex = 0;      // the first array element starts with 0 and is the default ration chosen

byte temperatureMode = Temperature_C;
byte temperatureSymbol = DEG_C_SYMBOL;

bool drawScreenNeeded = true;
volatile int lastKey = NO_KEY;

volatile int dirToGo = 0;
volatile unsigned long stepsToGo = 0;
volatile bool motorEnabled = false;

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
  
  Timer1.initialize(5);
  Timer1.stop();
  Timer1.attachInterrupt(actionMotor);

  displayMainMenu();
}                                    // end of setup function

void printIntroScreen()
{
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);                 // display flash screen
  lcd.print("Step Indexer 3.0");
  lcd.setCursor(0, 1);
  lcd.print("Ratio =");
  lcd.setCursor(8, 1);
  lcd.print(gear_ratio_top_array[gearratioindex]);
  lcd.setCursor(12, 1);
  lcd.print(":");
  lcd.print(gear_ratio_bottom_array[gearratioindex]);
}

void loop()                          // main loop services keystrokes
{ 
  int key = getKey();

  switch (cur_mode)                    // execute the keystroke
  {
  case mainmenu:                    // main menu
    doMainMenu(key);
    break;
  case tempmode:                    // call temps
    doTempMode(key);    
    break;
  case stepmode:                    // call steps
    doStepMode(key);    
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
}  

void printDebug()
{
  unsigned long now = millis();
  Serial.print(now);
  Serial.print(" enabled: ");
  Serial.print(motorEnabled);
  Serial.print(" stepsTogo: ");
  Serial.print(stepsToGo);
  Serial.print(" dir: ");
  Serial.println(dirToGo);
  last_debug_displayed = now;
}

void doMainMenu(int key)
{ 
  switch (key)
  {
  case LEFT_KEY:
  case UP_KEY:
    mode_select++;
    if (mode_select > numModes)  // wrap around menu
      mode_select = 1;      
    break;
  case RIGHT_KEY:
  case DOWN_KEY:
    mode_select--;
    if (mode_select < 1)         // wrap around menu
      mode_select = numModes;      
    break;
  case SELECT_KEY:         // user has picked a menu item
    setMode(mode_select);      
    break;
  }
}

void setMode(int mode)
{
  cur_mode = mode;
  displayScreen(mode);  
}

void doStepMode(int key)
{
  switch (key)
  {
  case UP_KEY:
    num_divisions++;      
    break;
  case DOWN_KEY:
    if (num_divisions > 1)
      num_divisions--;      
    break;
  case LEFT_KEY:
    cur_dir = CCW;
    stepsperdiv = (motorSteps / num_divisions);
    moveMotor(stepsperdiv, cur_dir);
    
    cur_pos--;
    if (cur_pos == (-num_divisions))
      cur_pos = 0;      
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

void doRunMode(int key)
{
  switch (key)
  {
  case UP_KEY:                        // bump up the speed
    if (motorspeeddelay >= SpeedDelayIncrement) {
      motorspeeddelay -= SpeedDelayIncrement;        
    }
    break;
  case DOWN_KEY:                      // bump down the speed
    motorspeeddelay += SpeedDelayIncrement;      
    break;
  case LEFT_KEY:                      // set direction
    cur_dir = CCW;      
    break;
  case RIGHT_KEY:                     // set other direction
    cur_dir = CW;      
    break;
  case SELECT_KEY:                   // user wants to stop
    motorspeeddelay = DefaultMotorSpeed;   // reset speed
    stopMotor();
    returnToMenu();
    return;
  }

  moveMotor(100000, cur_dir);
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
      if (gearratioindex > GearRatioMax - 1)  //wrap around if over the max
        gearratioindex = 0;
      motorSteps = (gear_ratio_top_array[gearratioindex] * Microsteps * StepsPerRevolution) / gear_ratio_bottom_array[gearratioindex];
      break;
    case DOWN_KEY:                        // reduce the number of steps
      --gearratioindex;
      if (gearratioindex < 0)
        gearratioindex = GearRatioMax - 1;    // wrap around if already at the bottom  
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

#pragma region Display routines

void displayMainMenu()
{
  lcd.print("Select Mode");
  lcd.setCursor(0, 1);
  lcd.print("Mode = ");
  lcd.setCursor(7, 1);
  switch (mode_select)
  {
  case(stepmode):
    lcd.print("Step");
    break;
  case(tempmode):
    lcd.print("Temp");
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
  }
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

void displayStepMode()
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
  lcd.setCursor(11, 0);
  if (cur_dir == CW)
    lcd.write(CW_SYMBOL);
  else
    lcd.write(CCW_SYMBOL);
  lcd.setCursor(0, 1);
  lcd.print("Speed = ");
  lcd.setCursor(8, 1);
  lcd.print((int)motorspeeddelay);
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

#pragma endregion

void displayScreen()
{
  displayScreen(cur_mode);
}

void displayScreen(int menunum)        // screen displays are here
{
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
  case stepmode:
    displayStepMode();
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
  noInterrupts();
  if (!stepsToGo) // Not executing any moves at present.
  {
    enableMotor();
    setDirection(dir);
    stepsToGo = steps;
    

    // Start execution.
    Timer1.restart();
    Timer1.start();
  }
  else
  {
    long stepsEffective = stepsToGo + (dirToGo == dir ? steps : -steps);
    if (stepsEffective < 0) // Adding steps in other direction needs reversing of the motor.
    {
      setDirection(dir);
      stepsToGo = abs(stepsEffective);
    }
    else
    {
      stepsToGo += steps;
    }
  }
  interrupts();

  printDebug();  
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
  digitalWrite(motorENABLEpin, HIGH);
}

void disableMotor()
{ 
  noInterrupts();  
  motorEnabled = false;
  digitalWrite(motorENABLEpin, LOW);
  interrupts();
}

void setDirection(int dir)
{
  dirToGo = dir;
  digitalWrite(motorDIRpin, dir);
}

volatile bool motorIRSBusy = false;
bool nextMotorPulseState = HIGH;

// Called from timer.
void actionMotor()
{
  if (motorIRSBusy) // Avoid nesting.
  {
    return;
  }

  motorIRSBusy = true;

  digitalWrite(LED_BUILTIN, nextMotorPulseState);
  nextMotorPulseState = !nextMotorPulseState;  
  if (nextMotorPulseState) // Pulse complete.
  {
    --stepsToGo;
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

unsigned long buttonLastRead = 0;
unsigned long keyLastHandled = 0;

int getKey()    // routine to return a valid keystroke
{
  
#ifdef SERIAL_INPUT
  int readKey = lastKey;
  lastKey = NO_KEY;
  return readKey;
#endif // !SERIAL_INPUT
  unsigned long now = millis();

  int readKey = readButton();
  if (readKey != lastKey)
  {
    if (now - buttonLastRead >= KeyDebounceDelayMs)
    {
      buttonLastRead = now;
      lastKey = readKey;
      return lastKey;
    }
  }

  // Send keypress no more than every KeyRepeatDelayMs.
  if (now - keyLastHandled >= KeyRepeatDelayMs)
  {
    keyLastHandled = now;
    return lastKey;
  }
  
  return NO_KEY;
}

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

int readButton()
{
  int key_in;
  
  analogRead(0); // Discard first ADC reading.
  key_in = analogRead(0);      // read ADC.

  // average values for my board were: 0, 144, 324, 505, 742
  // add approx 100 to those values to set range
  if (key_in > 850) return NO_KEY;
  if (key_in < 70)   return RIGHT_KEY;
  if (key_in < 250)  return UP_KEY;
  if (key_in < 450)  return DOWN_KEY;
  if (key_in < 650)  return LEFT_KEY;
  if (key_in < 850)  return SELECT_KEY;
}

