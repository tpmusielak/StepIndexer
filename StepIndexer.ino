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
3.0 Frozen   February 2017 by Tomasz P. Musielak
*/

//  First, include some files for the LCD display, and its motor

#include <Arduino.h>
#include <LiquidCrystal.h>

//  Next define your parameters about your motor and gearing

#define StepsPerRevolution 200  // Change this to represent the number of steps
//   it takes the motor to do one full revolution
//   200 is the result of a 1.8 degree per step motor
#define Microsteps 8            // Depending on your stepper driver, it may support
//   microstepping.  Set this number to the number of microsteps
//   that is set on the driver board.  For large ratios, you may want to 
//   use no microstepping and set this to 1.
#define GearRatio1top  3        // Change these three values to reflect any front end gearing you 
#define GearRatio1bottom 1      //   this is the bottom of the ratio - usually 1, as in 3 to 1, but you may change that here.
#define GearRatio2top  40       //   are using for three different devices.  If no gearing, then
#define GearRatio2bottom 1      //   40 to 1
#define GearRatio3top  90       //   define this value to be 1.  GearRatio1 is the default
#define GearRatio3bottom 1      //   90 to 1

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

//#define Celsius 1         // define only one of these, please!
#define Fahrenheit 1        // Fahrenheit is default

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

// create lcd display construct

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  //Data Structure and Pin Numbers for the LCD/Keypad

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

// create global variables

int    cur_mode = mainmenu;
int    mode_select = stepmode;
int    current_menu;
float  cur_angle = 0;
int    cur_key;
int    num_divisions = 0;
int    numjogsteps = JogStepsIncrement;
unsigned long stepsperdiv;
int    cur_pos = 0;
int    cur_dir = CW;
int    motorspeeddelay = DefaultMotorSpeed;
unsigned long motorSteps;
unsigned long gear_ratio_top_array[GearRatioMax] = { GearRatio1top,GearRatio2top,GearRatio3top };
unsigned long gear_ratio_bottom_array[GearRatioMax] = { GearRatio1bottom,GearRatio2bottom,GearRatio3bottom };
int    gearratioindex = 0;      // the first array element starts with 0 and is the default ration chosen

byte temperatureMode = Temperature_C;
byte temperatureSymbol = DEG_C_SYMBOL;

struct IndexerSettings
{

};

void setup()
{
  //  Create some custom characters for the lcd display
  byte c_CW[8] = { 0b01101,0b10011,0b10111,0b10000,0b10000,0b10000,0b10001,0b01110 }; // Clockwise
  byte c_CCW[8] = { 0b10110,0b11001,0b11101,0b00001,0b00001,0b00001,0b10001,0b01110 }; // CounterClockWise

  byte c_DegreeF[8] = { 0b01000,0b10100,0b01000,0b00111,0b00100,0b00110,0b00100,0b00100 }; //  degreeF
  byte c_DegreeC[8] = { 0b01000,0b10100,0b01011,0b00101,0b00100,0b00100,0b00101,0b00011 }; //  degreeC

  lcd.createChar(CW_SYMBOL, c_CW);
  lcd.createChar(CCW_SYMBOL, c_CCW);
  lcd.createChar(DEG_C_SYMBOL, c_DegreeC);
  lcd.createChar(DEG_F_SYMBOL, c_DegreeF);

  // begin program

  motorSteps = (gear_ratio_top_array[gearratioindex] * Microsteps * StepsPerRevolution) / gear_ratio_bottom_array[gearratioindex];
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);                 // display flash screen
  lcd.print("Step Indexer 2.3");
  lcd.setCursor(0, 1);
  lcd.print("Ratio =");
  lcd.setCursor(8, 1);
  lcd.print(gear_ratio_top_array[gearratioindex]);
  lcd.setCursor(12, 1);
  lcd.print(":");
  lcd.print(gear_ratio_bottom_array[gearratioindex]);
  delay(1500);                        // wait a few secs
  lcd.clear();

  pinMode(motorSTEPpin, OUTPUT);     // set pin 3 to output
  pinMode(motorDIRpin, OUTPUT);      // set pin 2 to output
  pinMode(motorENABLEpin, OUTPUT);   // set pin 11 to output
  digitalWrite(motorENABLEpin, HIGH); // power up the motor and leave it on

  displayScreen(cur_mode);           // put up initial menu screen

}                                    // end of setup function

void loop()                          // main loop services keystrokes
{
  int exitflag;
  int numjogsteps;

  displayScreen(cur_mode);            // display the screen
  cur_key = getRealKey();           // grab a keypress
  switch (cur_mode)                    // execute the keystroke
  {
  case mainmenu:                    // main menu
    switch (cur_key)
    {
    case UP_KEY:
      mode_select++;
      if (mode_select > numModes)  // wrap around menu
        mode_select = 1;
      break;
    case DOWN_KEY:
      mode_select--;
      if (mode_select < 1)         // wrap around menu
        mode_select = numModes;
      break;
    case LEFT_KEY:           // left and right keys do nothing in main menu
      break;
    case RIGHT_KEY:
      break;
    case SELECT_KEY:         // user has picked a menu item
      cur_mode = mode_select;
      break;
    }
    break;
  case tempmode:                    // call temps
    dotempmode(cur_key);
    cur_mode = mainmenu;
    break;
  case stepmode:                    // call steps
    doStepMode(cur_key);
    cur_mode = mainmenu;
    break;
  case anglemode:                   // call angles
    doAngleMode(cur_key);
    cur_mode = mainmenu;
    break;
  case runmode:                     // call run  
    doRunMode(cur_key);
    cur_mode = mainmenu;
    break;
  case jogmode:                    // call jog
    doJogMode(cur_key);
    cur_mode = mainmenu;
    break;
  case ratiomode:                 // call ratio
    doRatioMode(cur_key);
    cur_mode = mainmenu;
    break;
  } // end of mode switch
}   // end of main loop

void doStepMode(int tmp_key)
{
  int breakflag = 0;
  displayScreen(stepmode);
  while (breakflag == 0)
  {
    switch (tmp_key)
    {
    case UP_KEY:
      num_divisions++;
      break;
    case DOWN_KEY:
      if (num_divisions > 0)
        num_divisions--;
      break;
    case LEFT_KEY:
      cur_dir = CCW;
      stepsperdiv = (motorSteps / num_divisions);
      moveMotor(stepsperdiv, cur_dir, movesteps);
      delay(ScreenPause);   //pause to inspect the screen
      cur_pos--;
      if (cur_pos == (-num_divisions))
        cur_pos = 0;
      break;
    case RIGHT_KEY:
      cur_dir = CW;
      stepsperdiv = (motorSteps / num_divisions);
      moveMotor(stepsperdiv, cur_dir, movesteps);
      delay(ScreenPause);   // pause to inspect the screen
      cur_pos++;
      if (cur_pos == num_divisions)
        cur_pos = 0;
      break;
    case SELECT_KEY:
      cur_pos = 0;                       // reset position
      num_divisions = 0;                 // reset number of divisions
      return;
      break;
    }
    displayScreen(stepmode);
    tmp_key = getRealKey();
  }
  return;
}
void doAngleMode(int tmp_key)
{
  int breakflag = 0;
  displayScreen(anglemode);
  while (breakflag == 0)
  {
    switch (tmp_key)
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
      moveMotor(stepsperdiv, cur_dir, moveangle);
      delay(ScreenPause);   //pause to inspect the screen
      break;
    case RIGHT_KEY:
      cur_dir = CW;
      stepsperdiv = ((motorSteps * cur_angle) / 360);
      moveMotor(stepsperdiv, cur_dir, moveangle);
      delay(ScreenPause);   //pause to inspect the screen
      break;
    case SELECT_KEY:
      cur_angle = 0;                       // reset angle to default of zero
      return;
      break;
    }
    displayScreen(anglemode);
    tmp_key = getRealKey();
  }
  return;
}

bool keyPressed()
{
  return analogRead(AnalogKeyPin) < 850;
}

void doRunMode(int tmp_key)
{
  bool breakflag = false;
  delay(100);                              // wait for keybounce from user's selection
  cur_dir = CW;                            // initially, clockwise
  displayScreen(runmode);                  // show the screen

  while (breakflag == false)               // cycle until Select Key sets flag
  {
    moveMotor(1, cur_dir, 0);              // move motor 1 step
    if (keyPressed())                      // if a keypress is present       {
    {
      cur_key = getRealKey();              // then get it
      switch (cur_key)                     // and honor it
      {
      case UP_KEY:                        // bump up the speed
        if (motorspeeddelay >= SpeedDelayIncrement) {
          motorspeeddelay -= SpeedDelayIncrement;
          displayScreen(runmode);
        }
        break;
      case DOWN_KEY:                      // bump down the speed
        motorspeeddelay += SpeedDelayIncrement;
        displayScreen(runmode);
        break;
      case LEFT_KEY:                      // set direction
        cur_dir = CCW;
        displayScreen(runmode);
        break;
      case RIGHT_KEY:                     // set other direction
        cur_dir = CW;
        displayScreen(runmode);
        break;
      case SELECT_KEY:                   // user wants to stop
        motorspeeddelay = DefaultMotorSpeed;   // reset speed   
        breakflag = 1;                   // fall through to exit
      }
    }
  }
}
void dotempmode(int test_key)
{
  while (1)
  {
    if (test_key == SELECT_KEY) return;     // all we do here is wait for a select key
    test_key = getRealKey();
  }
  return;                                   // to exit
}

void doJogMode(int tmp_key)
{
  bool breakflag = false;

  numjogsteps = JogStepsIncrement;
  while (breakflag == false)
  {
    if (breakflag == false)
    {
      displayScreen(jogmode);
      tmp_key = getRealKey();
    }
    switch (tmp_key)
    {
      case UP_KEY:                          // bump the number of steps
        numjogsteps += JogStepsIncrement;
        break;
      case DOWN_KEY:                        // reduce the number of steps
        if (numjogsteps > JogStepsIncrement)
          numjogsteps -= JogStepsIncrement;
        break;
      case LEFT_KEY:                        // step the motor CCW
        moveMotor(numjogsteps, CCW, 0);
        break;
      case RIGHT_KEY:                       // step the motor CW
        moveMotor(numjogsteps, CW, 0);
        break;
      case SELECT_KEY:                      // user want to quit
        breakflag = true;
        break;
    }
  }

  numjogsteps = JogStepsIncrement;
  cur_mode = mainmenu;                 // go back to main menu
  return;
}

void doRatioMode(int tmp_key)
{
  int breakflag = 0;

  displayScreen(ratiomode);
  for (; breakflag == 0;)
  {
    switch (tmp_key)
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
      breakflag = 1;
      break;
    }
    if (breakflag == 0)
    {
      displayScreen(ratiomode);
      tmp_key = getRealKey();
    }
  }
  numjogsteps = JogStepsIncrement;
  cur_mode = mainmenu;                 // go back to main menu
  return;
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

void pulseMotor(int dir)
{
  digitalWrite(motorDIRpin, dir);
  digitalWrite(motorSTEPpin, HIGH);   //pulse the motor
  delay(pulsewidth);
  digitalWrite(motorSTEPpin, LOW);
}

void moveMotor(unsigned long steps, int dir, int type)
{
  unsigned long i;

  if (type == movesteps)
  {
    displayScreen(movesteps);
  }
  else if (type == moveangle)
  {
    displayScreen(moveangle);
  }

  for (i = 0; i < steps; i++)
  {
    pulseMotor(dir);
    if (type == movesteps)             // in this mode display progress
    {
      lcd.setCursor(10, 1);
      lcd.print(i);
    }
    if (type == moveangle)             // in this mode display progress
    {
      lcd.setCursor(7, 0);
      lcd.print(i);
    }
    delay(motorspeeddelay);            // wait betweeen pulses
  }
  return;
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

int getRealKey(void)    // routine to return a valid keystroke
{
  int trial_key = 0;
  while (trial_key < 1)
  {
    trial_key = readButton();
  }
  delay(200);             // 200 millisec delay between user keys
  return trial_key;
}

int readButton()     // routine to read the LCD's buttons
{
  int key_in;
  delay(ADSettleTime);         // wait to settle
  key_in = analogRead(0);      // read ADC once
  delay(ADSettleTime);         // wait to settle
                 // average values for my board were: 0, 144, 324, 505, 742
                 // add approx 100 to those values to set range
  if (key_in > 850) return NO_KEY;
  if (key_in < 70)   return RIGHT_KEY;
  if (key_in < 250)  return UP_KEY;
  if (key_in < 450)  return DOWN_KEY;
  if (key_in < 650)  return LEFT_KEY;
  if (key_in < 850)  return SELECT_KEY;
}
