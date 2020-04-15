
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
#define PI 3.1415926535897932384626433832795

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
volatile bool extInterrupt = false;
// volatile bool timer1Interrupt = false;

////////////////////////////////////////////////////////
// Setting OCR from Serial input
char firstDigit = 0;  // For checking start of OCR stream
char secondDigit = 0;
String compareReg;  // Store incoming OCR value temporarily
unsigned long OCR;
unsigned long OCR_125 = 15999; // OCR for a frequency of 125 Hz
volatile bool serFlag = false;
String flushInputBuffer;

////////////////////////////////////////////////////////
// Handshake variables
char shakeDigit =0; // individual bit of handshake word
byte indexShake = 0; // index of handshake input word
String shakeInput; // 3 bit password to assign pump name/position
String shakeKey = "RHS"; // RHS = 4, TOP = 5, LHS = 6
bool shakeFlag = false;

////////////////////////////////////////////////////////
// Pressure sensor variables
MS5803 sensor(ADDRESS_LOW);//CSB pin pulled low, so address low
double pressureAbs = 1000.00; // Initial value
int pressThresh = 5;//mbar
int pressMAX = 2000;
double pressSetpoint = 800.00;//mbar
bool pressFlag = false;
int pressureError;
int sampDiv = 6; // Factor to divide Timer2 frequency by
volatile int sampCount = 0;
volatile bool sampFlag = false;

////////////////////////////////////////////////////////
// Stepper variables
int stepsPMM = 100;
int limitSteps = stepsPMM*2; // number of pulses for 1 mm
int motorState = 0;
volatile int stepCount;
String stepRecv;
int stepIn;
int stepInPrev;
int stepError;
bool motorDirection = HIGH;
// number of steps at max step frequency in 100 Hz timestep
int fSamp = 100;
int stepsPerLoop = 2000/fSamp;
int tSampu = 10000; // (1/fSamp)e6 Time between samples in microseconds
unsigned long tStep; // 
unsigned long timeSinceStep;
unsigned long timeNow;
unsigned long timeAtStep;

////////////////////////////////////////////////////////
// Actuator geometry
float S = 50.0; //Eq. triangle length in mm
float L0 = S/(1.0 - 2.0/PI); // flat length for contraction = S
float W = 30.0; // width of muscle in mm
float numLs = 10; // number of subdivisions in muscle
float As = PI*pow(15.0, 2.0); // piston area in mm^2
float factV = (W*pow(L0 , 2.0))/(2.0*numLs);
float maxV = factV*(2.0/PI); // volume in mm^3 when fully actuated
// steps to fill actuator rounded down, minus 1 timestep's worth
int maxSteps = ((maxV/As)*stepsPMM - stepsPerLoop); 


void setup() {
  Wire.begin();
  //Serial configuration
  Serial.begin(115200);
  // Wait here until serial port is opened
  while (!Serial){
    ;
  }

  //Sensor startup - see https://github.com/sparkfun/MS5803-14BA_Breakout/
  sensor.reset();
  sensor.begin();
  delay(1000);
  pressureAbs = sensor.getPressure(ADC_4096);
  // Serial.print("Absolute pressure = ");
  // Serial.print(pressureAbs);
  // Serial.println(" mbar.");

  // Disable all interrupts
  noInterrupts();
  // External interrupt config.
  // 3k3 Pull-up resistors used, switch connects pin to gnd, so falling edge or LOW trigger.
  pinMode(fwdIntrrptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(fwdIntrrptPin), forwardInterrupt, LOW);
  pinMode(bwdIntrrptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(bwdIntrrptPin), backwardInterrupt, LOW);

  // Configuration of DRV8825 driver pins
  pinModeFast(directionPin, OUTPUT);
  pinModeFast(stepPin, OUTPUT);
  pinModeFast(enablePin, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  //Set all microstepping pins (M0 - M2) high for 32 microsteps
  //M2M1M0 = 111 = 32, 100 = 16, 011 = 8, 010 = 4, 001 = 2, 000 = 1
  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  //pinMode(testPin, OUTPUT);
  // Start with motor diasbled
  digitalWriteFast(enablePin, HIGH);

  //Initialise timer 2 - prescaler = 1024 gives ~61 - 15625 Hz range
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TCCR2A |= (1 << WGM21);   //Set CTC mode
  // Set OCR2A to 124 for 125 Hz timer, where serial
  // can be read and pressure taken every 6th cycle.
  // desired frequency = 16Mhz/(prescaler*(1+OCR)) 
    // 16e6/(1024*(255+1))= 61.035 Hz
    // 16e6/(1024*(124+1)) = 125 Hz
  OCR2A = 156;//124
  // Set CS22, CS21 and CS20 bits for 1024 prescaler
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);   // Turn on
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}


// Internal interrupt service routine, timer 2 overflow
ISR(TIMER2_COMPA_vect){
  interrupts(); //re-enable interrupts
  // This will be called at 125 Hz, so every sampDiv times set sampling flag to reduce the frequency.
  if (sampCount == sampDiv){
    sampFlag = true;
    sampCount = 1;
  }
  else {
    sampCount += 1;
  }
  serFlag = true;
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
    ;
    // Move piston back 'manually'
    // digitalWriteFast(directionPin, LOW);
    // moveMotor();
    // Disable motor
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
      //Stop timer/counter 1
      // TCCR1B &= (0 << CS11);
      // TCNT1 = 0;
      break;
    case 1:
      //Move motor forwards
      motorDirection = HIGH;
      digitalWriteFast(directionPin, HIGH);
      //Serial.println("INCREASE PRESSURE");
      break;
    case 2:
      //Move motor back
      motorDirection = LOW;
      digitalWriteFast(directionPin, LOW);
      //Serial.println("DECREASE PRESSURE");
      break;
    default:
      //Just in case nothing matches, stop timer/counter
      break;
  }
}


//External interrupt service function
//Set both step number and the counter used in for loop of stepping function to zero.
void forwardInterrupt() {
  //Stop other interrupts
  noInterrupts();
  // Serial.println("External Interrupt - Front");
  extInterrupt = true;
  //Change setpoint to currrent pressure to prevent motion
  pressSetpoint = pressureAbs;
  stepCount -= limitSteps; //Keep track of volume change after moveMotor() call
  digitalWriteFast(directionPin, LOW);
  moveMotor();
  interrupts();
}


//External interrupt service Call function
//Set both step number, counter used for loop of stepping function to zero.
void backwardInterrupt() {
  //Stop other interrupts
  noInterrupts();
  // Serial.println("External Interrupt - Back");
  extInterrupt = true;
  //Change setpoint to currrent pressure to prevent motion
  pressSetpoint = pressureAbs;
  stepCount += limitSteps; //Keep track of volume change after moveMotor() call
  digitalWriteFast(directionPin, HIGH);
  moveMotor();
  interrupts();
}


void moveMotor() {
  for (volatile unsigned long moveBackB = 0; moveBackB < limitSteps; moveBackB++) {
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
    Serial.println(shakeInput); // CHANGE SERIAL PRINTS TO SERIAL WRITES?
  }
  if (shakeInput == shakeKey){
    shakeFlag = true;
    // Enable the motor after handshaking
    digitalWriteFast(enablePin, LOW);
    shakeInput = "";
    // Serial.println(shakeFlag);
  }
  delayMicroseconds(50000);
  // indexShake = 0;
}


//Function to read input in serial monitor and set the new desired pressure.
void readSerial() {
  while (Serial.available() > 0) {
    firstDigit = Serial.read();

    // Control code sends capital S to receive stepCount
    // Capital S in ASCII is 83, so check for that:
    if (firstDigit == 83) {
      stepRecv = Serial.readStringUntil('\n');
      if (stepRecv == "Closed"){
        // Disable the motor
        digitalWriteFast(enablePin, HIGH);
        while(1){
          ;
        }
      }
      else{
        stepIn = stepRecv.toInt();
        if (stepIn > maxSteps){
          stepIn = maxSteps;
        }
        stepError = stepIn - stepCount;
        if (abs(stepError) > 0){
          // If piston not at desired position,
          // work out timeStep so that piston reaches
          // after 1/fSamp seconds
          tStep = floor(tSampu/abs(stepError));
          if (tStep < 500){
            tStep = 500; // Limit fStep to 2 kHz
          }
        }
        else{
          // Set tStep to unattainably large value
          // so no steps are made.
          tStep = 20000;
        }
        Serial.println(stepCount);
        // Do something to compare received position and actual position
        // Serial.println(stepCount);
        // Serial.println(stepIn);
      }
    }

    // Check for capital P:
    else if (firstDigit == 80) {
      Serial.println(pressureAbs);
    }

    // Check for capital O:
    else if (firstDigit == 79) {
      secondDigit = Serial.read();
      // Check for positive sign
      if (secondDigit == 43){
        // Set direction as forwards
        // motorDirection = HIGH;
        // digitalWriteFast(directionPin, HIGH);
        ;
      }
      // Else check for negative sign
      else if (secondDigit == 45){
        // Set direction as backwards
        // motorDirection = LOW;
        // digitalWriteFast(directionPin, LOW);
        ;
      }
      compareReg = Serial.readStringUntil('\n');
      OCR = long(compareReg.toFloat());
      Serial.println(stepError);
    }

    else {
      flushInputBuffer = Serial.readStringUntil('\n');
    }
  }
}

void loop() {

 while (shakeFlag == false){
   handShake();
   stepCount = 2168;
 }

  // On startup, pull -ve pressure and zero the volume
  if (pressFlag == false){  // Change this to a while loop later when it works
    // pressInitZeroVol();
    pressFlag = true;
    stepCount = 2168;
  }
  // NEED TO MOVE TO HOME POSITION before entering loop in controlSystem.py

  // Read in new position value with Timer2 interrupt
  if (serFlag == true){
    stepInPrev = stepIn;
    readSerial(); // Read stepIn, send stepCount
    readSerial(); // Read OCR
    serFlag = false;
  }

  // Call overpressure protection every 6th Timer2 interrupt
  if (sampFlag == true) {
    pressureProtect();
    sampFlag = false;
  }

  // Do something if gantry hits limit switches
  if (extInterrupt == true) {
    // Turn off interrupts
    TCCR2B = 0;
    // Turn off timer1 clock input
    // Disable motor drivers
    digitalWriteFast(enablePin, HIGH);
    pressureProtect();
    // Serial.println(pressureAbs);
    shakeFlag = false;
    // Need to handshake again to reactivate
    while(shakeFlag == false){
      flushInputBuffer = Serial.readStringUntil('\n');
      Serial.println("Limit reached.");
      flushInputBuffer = Serial.readStringUntil('\n');
      Serial.println("Handshake to activate.");
      handShake();
    }
    // Turn on pressure interrupt
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
    extInterrupt = false;
  }

  // Step the motor if enough time has passed.
  timeNow = micros();
  timeSinceStep = timeNow - timeAtStep;
  if (tStep == 20000){
    ;// Do nothing.
  }
  else if (timeSinceStep >= tStep){
    if (stepError > 0){
      if (stepCount < maxSteps){
        motorDirection = HIGH;
        digitalWriteFast(directionPin, HIGH);
        digitalWriteFast(stepPin, HIGH);
        digitalWriteFast(stepPin, LOW);
        stepCount += 1;
      }
    }
    else if (stepError < 0){
      if (stepCount > 0){
        motorDirection = LOW;
        digitalWriteFast(directionPin, LOW);
        digitalWriteFast(stepPin, HIGH);
        digitalWriteFast(stepPin, LOW);
        stepCount -= 1;
      }
    }
    stepError = stepIn - stepCount;
    timeAtStep = micros();
  }

}
