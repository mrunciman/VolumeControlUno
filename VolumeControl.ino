
/*Pressure control sketch for syringe pump.
   Arduino Mega 2560 with DRV8825 drivers

   MS5803_14BA pressure sensor via I2C

   Timer1 overflows at 20 Hz, calling function to read pressure and setting forward, backward or no motion.
   Timer2 overflows at 2000 Hz, sending pulse to stepper motor

   Stepper is stopped by setting control register to 0, stopping pulses: TCCR2B = 0;

   External interrupts on limit switches have priority and will reverse motor.

   New pressure setpoint can be typed into serial, in units of mbar.

*/


#include <math.h>
#include <Wire.h>
//Sensor library - https://github.com/sparkfun/MS5803-14BA_Breakout/
#include <SparkFun_MS5803_I2C.h>
#include <digitalWriteFast.h>

//Connect Stepper motor to motor driver
//For step count:
//Mapping from step to 1 revolution = 200 steps
//32 microsteps per step, so 6400 steps per revolution
//Lead = start*pitch      Lead screw is 4 start, 2 mm pitch, therefore Lead = 8
//Steps/mm = (StepPerRev*Microsteps)/(Lead) = 800 steps/mm
//Set M0, M1, M2 to set microstep size
#define directionPin 6
#define stepPin 5
#define enablePin 4
#define M0 9
#define M1 7
#define M2 8
//#define testPin 11
//Set external interrupt pin numbers
//Possible interrupt pins on Uno: 2, 3 - 18, 19 on Mega
//First and second (Xmin and Xmax) end stop headers on Ramps Board
const byte fwdIntrrptPin = 2;
const byte bwdIntrrptPin = 3;
volatile bool extInterrupt;
volatile bool timer1Interrupt =  false;

char setpointDigit = 0; // individual bits of set point input
byte indexPress = 0; // index of input pressure byte
char setpointInput[7]; // input set point pressure in mbar
char flushInputBuffer[20];
volatile bool motorDirection;

char shakeDigit =0; // individual bit of handshake word
byte indexShake = 0; // index of handshake input word
String shakeInput; // 3 bit password to assign pump name/position
String shakeKey = "RHS";
bool shakeFlag = false;

//Instantiate sensor
MS5803 sensor(ADDRESS_LOW);//CSB pin pulled low, so address low
//Create variables to store results
//float tempCelsius;
volatile double pressureAbs = 1000.00;
int pressThresh = 5;//mbar
int pressMAX = 2000;
double pressSetpoint = 800;//mbar
bool pressFlag = false;
String stepFreq;
int compareReg;

//Internal interrupt variables
int motorState = 0;
int pressureError;
long mainFreq = 20;//Hz     20 Hz is maximum sampling rate
long timerOneCount;

volatile int stepCount = 0;

void setup() {
  Wire.begin();
  //Serial configuration
  Serial.begin(115200);
  // Wait here until serial port is opened
  while (!Serial){
    ;
  }

  // Disable all interrupts
  noInterrupts();
  //Interrupt config.
  //3k3 Pull-up resistors used, switch connects pin to gnd, so falling edge or LOW trigger.
  pinMode(fwdIntrrptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(fwdIntrrptPin), forwardInterrupt, LOW);
  pinMode(bwdIntrrptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(bwdIntrrptPin), backwardInterrupt, LOW);
  extInterrupt = false;

  // Configuration of DRV8825 driver pins
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  //Set all microstepping pins (M0 - M2) high for 32 microsteps
  //M2M1M0 = 111 = 32, 100 = 16, 011 = 8, 010 = 4, 001 = 2, 000 = 1
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  //pinMode(testPin, OUTPUT);

  // Initialize timer 1 - for 20 Hz pressure sampling
  TCCR1A = 0;
  TCCR1B = 0;
  // Set timerOneCount to the correct value for our interrupt interval
  // preload timer (65536 - 16MHz/(prescaler*freq))
  timerOneCount = 65536 - round(16000000 / (256 * mainFreq));
  TCNT1 = timerOneCount;  // preload timer
  TCCR1B |= (1 << CS12);    // prescaler = 256 - on timer 1, pull high clock select bit 2
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

  //Initialise timer 2 - prescaler = 1024 gives ~61 - 15625 Hz range
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS22, CS21 and CS20 bits for 1024 prescaler
  //TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);   // Leave timer stopped just now (TCCR2B = 0;)
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  // Set OCR, which TCNT counts up to.
  // desired frequency = 16Mhz/(prescaler*(1+OCR))
  // OCR = 16000000/(1024*stepFreq) - 1
  OCR2A = 0;
  interrupts();             // enable all interrupts

  //Sensor startup - see https://github.com/sparkfun/MS5803-14BA_Breakout/
  sensor.reset();
  sensor.begin();
  delay(1000);
  pressureAbs = sensor.getPressure(ADC_4096);
  Serial.print("Absolute pressure = ");
  Serial.print(pressureAbs);
  Serial.println(" mbar.");
}


// Internal interrupt service routine, timer 1 overflow
ISR(TIMER1_OVF_vect){
  interrupts(); //re-enable interrupts
  // preload the timer each cycle
  TCNT1 = timerOneCount;
  timer1Interrupt = true;
}


// Internal interrupt service routine, timer 2 overflow
ISR(TIMER2_COMPA_vect){
  interrupts(); //re-enable interrupts

  //toggle step pin to create pulse
  digitalWriteFast(stepPin, HIGH);
  digitalWriteFast(stepPin, LOW);
  //For volume change calculation:
  //If moving forwards, increase step counter
  if (motorDirection == false) {
    stepCount += 1;
  }
  //If moving backwards, decrease step counter
  else if (motorDirection == true) {
    stepCount -= 1;
  }
}


void pressureProtect() {
  pressureAbs = sensor.getPressure(ADC_4096);
  // Filter out false readings
  // if (pressureAbs < 0){
  //   pressureAbs = 2500;
  // }
  // else if (pressureAbs > 2500){
  //   pressureAbs = 2499;
  // }

  //Do something if pressure exceeds some limit:
  // Stop motor if pressure reaches maximum allowed value
  if (pressureAbs > pressMAX){
    // TCCR2B = 0;
    ;
  }
  // Serial.print("Absolute pressure = ");
  // Serial.print(pressureAbs);
  // Serial.println(" mbar.");
}


void pressInitZeroVol() {
  // Loop here until predsure is within threshold? 

  //Set state for motor motion based on comparison of pressure signal with setpoint
  //If within pressThresh mbar, don't move motor
  pressureAbs = sensor.getPressure(ADC_4096);
  if (pressureAbs < 0) {
    pressureAbs = 2500;
  }
  else if (pressureAbs > 2500) {
    pressureAbs = 2499;
  }
  //Serial.print("Absolute pressure = ");
  //Serial.println(pressureAbs);
  //Serial.println(" mbar.");

  pressureError = pressureAbs - pressSetpoint;
  if (abs(pressureError) <= pressThresh) {
    motorState = 0;
  }
  //If pressure lower than setpoint, move motor forwards
  else if (pressureError < -pressThresh) {
    motorState = 1;
  }
  //If higher than setpoint, move motor back
  else if (pressureError > pressThresh) {
    motorState = 2;
  }
  switch (motorState) {
    case 0:
      //Stop timer/counter 2
      TCCR2B = 0;
      break;
    case 1:
      //Move motor forwards
      motorDirection = false;
      digitalWriteFast(directionPin, motorDirection);
      // CHANGE TIMER SET UP FOR CTC setup
      // Set CS22, CS21 and CS20 bits for 1024 prescaler
      TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);    
      //Serial.println("INCREASE PRESSURE");
      break;
    case 2:
      //Move motor back
      motorDirection = true;
      digitalWriteFast(directionPin, motorDirection);
      // Set CS22, CS21 and CS20 bits for 1024 prescaler
      TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
      //Serial.println("DECREASE PRESSURE");
      break;
  }
}


//External interrupt service function
//Set both step number and the counter used in for loop of stepping function to zero.
void forwardInterrupt() {
  //Stop timer/counter 2 - stop motor
  TCCR2B = 0;
  //Stop other interrupts
  noInterrupts();
//  Serial.println("External Interrupt - Front");
  extInterrupt = true;
  //Set serial input to zero
  for (int i = 0; i < sizeof(setpointInput);  ++i ) {
    setpointInput[i] = (char)0;
  }
  //Change setpoint to currrent pressure to prevent motion
  pressSetpoint = pressureAbs;
  stepCount -= 400; //Keep track of volume change after moveMotor() call
  digitalWriteFast(directionPin, true);
  moveMotor();
  interrupts();
}


//External interrupt service Call function
//Set both step number, counter used for loop of stepping function to zero.
void backwardInterrupt() {
  //Stop timer/counter 2 - stop motor
  TCCR2B = 0;
  //Stop other interrupts
  noInterrupts();
//  Serial.println("External Interrupt - Back");
  extInterrupt = true;
  //Set serial input to zero - from pressure controller
  for (int i = 0; i < sizeof(setpointInput);  ++i ) {
    setpointInput[i] = (char)0;
  }
  //Change setpoint to currrent pressure to prevent motion
  pressSetpoint = pressureAbs;
  stepCount += 400; //Keep track of volume change after moveMotor() call
  digitalWriteFast(directionPin, false);
  moveMotor();
  interrupts();
}


void moveMotor() {
  for (volatile unsigned long moveBackB = 0; moveBackB < 400; moveBackB++) {
    //A step is executed when a rising edge is detected on Step motor input
    digitalWriteFast(stepPin, HIGH);
    digitalWriteFast(stepPin, LOW);
    delayMicroseconds(500);
  }
}


//Function to read input from python script and confirm pump position (TOP, LHS, RHS).
void handShake() {
  // shakeInput = "";
  while (Serial.available() > 0) {
    shakeInput = Serial.readStringUntil('\n');
  }
  if (shakeInput!=""){
    Serial.println(shakeInput); // CHANGE SERIAL PRINTS TO SERIAL WRITES
  }
  if (shakeInput == shakeKey){
    shakeFlag = true;
    // Serial.println(shakeFlag);
  }
  indexShake = 0;
}


//Function to read input in serial monitor and set the new desired pressure.
void readSerial() {
  while (Serial.available() > 0) {
    setpointDigit = Serial.read();
    // Control code sends capital S to receive stepCount
    // Capital S in ASCII is 83, so check for that:
    if (setpointDigit == 83) {
      Serial.println(stepCount);
    }
    // Check for capital P:
    else if (setpointDigit == 80) {
      Serial.println(pressureAbs);
    }
    // Check for capital O:
    else if (setpointDigit == 79) {
      stepFreq = Serial.readStringUntil('\n');
      // Do this calc in the python script
      // stepFreq = stepFreq.toFloat();
      // compareReg = round((16000000/(1024*stepFreq.toFloat()))-1);
      OCR2A = compareReg;
      TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
      Serial.println(stepFreq);
      Serial.println(compareReg);
    }
    else {
      flushInputBuffer[indexPress] = Serial.read();
      indexPress++;
    }
  }
  indexPress = 0;
}


void loop() {

 while (shakeFlag == false){
   handShake();
   shakeInput = "";
 }

  // On startup, pull -ve pressure and zero the volume
  if (pressFlag == false){  // Change this to a while loop later when it works
    // pressInitZeroVol();
    stepCount = 0;
    pressFlag = true;
  }

  // Call overpressure protection function on 20Hz Timer1 interrupt
  if (timer1Interrupt == true) {
    pressureProtect();
    timer1Interrupt = false;
  }

  // Do something if gantry hits limit switches
  if (extInterrupt == true) {
    pressureProtect();
    Serial.println(pressureAbs);
    extInterrupt = false;
  }

  // Read in new frequency value
  readSerial();
}
