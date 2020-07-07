
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
// #include <digitalWriteFast.h>
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

volatile int pumpState;

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
unsigned long OCR;
volatile bool serFlag = false;
String flushInputBuffer;

////////////////////////////////////////////////////////
// Handshake variables
bool shakeFlag = false;
String shakeInput; // 3 bit password to assign pump name/position
char shakeKey[5] = "LHS"; // RHS = 4, TOP = 5, LHS = 6

////////////////////////////////////////////////////////
// Pressure sensor variables
MS5803 sensor(ADDRESS_LOW);//CSB pin pulled low, so address low
double pressureAbs = 1000.00; // Initial value
int pressThresh = 10;//mbar
int pressMAX = 2500;
volatile double pressSetpoint = 800.00;//mbar
bool pressFlag = false;
int pressureError;
int sampDiv = 6; // Factor to divide Timer2 frequency by
volatile int sampCount = 0;
volatile bool sampFlag = false;
int OCR_2p5mmps = 7999;
// Calibration
int stateCount = 0;
int startTime;
int stableTime = 5000; // time in milliseconds reqd for pressure to be at setpoint

////////////////////////////////////////////////////////
// Stepper variables
int stepsPMM = 100;
int limitSteps = stepsPMM*2; // number of pulses for 1 mm
int prevMotorState = 0;
int motorState = 3;
volatile int stepCount = 0;
String stepRecv;
int stepIn;
int stepError = 0;
bool motorDirection = HIGH;
int fSamp = 20;
int stepsPerLoop = 2000/fSamp; // number of steps at max step frequency in fSamp Hz timestep
unsigned long tSampu = 1000000/fSamp; // (1/fSamp)e6 Time between samples in microseconds
unsigned long oneHour = 3600000000;
unsigned long tStep = oneHour;
unsigned long timeSinceStep = 0;
unsigned long timeNow;
unsigned long timeAtStep;
unsigned long writeTime;
unsigned long tStep2k = 500; // When tStep equals 500 ms, 2 kHz pulse achieved.

////////////////////////////////////////////////////////
// Messages
char data[40]; // Char array to write stepNo, pressure and time into
char endByte[3] = "E";
char disableMsg[3] = "D ";
char limitHit[3] = "L ";

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
  pressureAbs = sensor.getPressure(ADC_2048);

  // Disable all interrupts
  noInterrupts();
  // External interrupt config.
  // 3k3 Pull-up resistors used, switch connects pin to gnd, so falling edge or LOW trigger.
  pinMode(fwdIntrrptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(fwdIntrrptPin), forwardInterrupt, LOW);
  pinMode(bwdIntrrptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(bwdIntrrptPin), backwardInterrupt, LOW);

  // Configuration of DRV8825 driver pins
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
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
  digitalWrite(enablePin, HIGH);

  // Set up Timer1 for volume calibration procedure.
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  TCNT1 = 0;
  TIMSK1 = 0;
  TCCR1B |= (1 << WGM12);  //Set CTC mode
  TIMSK1 |= (1 << OCIE1A);  //enable timer compare interrupt
  TCCR1B &= (0 << CS10); // Turn off clock source
  TCCR1B &= (0 << CS11); 
  TCCR1B &= (0 << CS12);
  // TCCR1B |= (1 << CS11);  // Turn on clock with prescaler 8 
  // OCR1A = 65535;

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
  OCR2A = 124;//124 for 125 Hz and 156 for 100 Hz
  // Set CS22, CS21 and CS20 bits for 1024 prescaler
  // TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);   // Turn on
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

// ISR Timer1 used for calibration phase
ISR(TIMER1_COMPA_vect){
  interrupts();
  digitalWrite(stepPin, HIGH);
  digitalWrite(stepPin, LOW);
}

// Internal interrupt service routine, timer 2 overflow
ISR(TIMER2_COMPA_vect){
  interrupts(); //re-enable interrupts
  // This will be called at 125 Hz, so every sampDiv times set sampling flag, reducing the frequency.
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
  pressureAbs = sensor.getPressure(ADC_2048);
  // Filter out false readings
  // if (pressureAbs < 0) {
  //   pressureAbs = pressMAX;
  // }
  // else if (pressureAbs > pressMAX) {
  //   pressureAbs = pressMAX-1;
  // }
  //Do something if pressure exceeds some limit:
  if (pressureAbs > pressMAX){
    extInterrupt = true;
  }
}


void pressInitZeroVol() {
  //Set state for motor motion based on comparison of pressure signal with setpoint
  //If within pressThresh mbar, don't move motor
  pressureAbs = sensor.getPressure(ADC_2048);
  // if (pressureAbs < 0) {
  //   pressureAbs = pressMAX;
  // }
  // else if (pressureAbs > pressMAX) {
  //   pressureAbs = pressMAX-1;
  // }

  pressureError = pressureAbs - pressSetpoint;
  prevMotorState = motorState;
  // Assign motor state based on pressure error
  if (abs(pressureError) <= pressThresh) {
    motorState = 0;
    // Increment counter if previous state was also zero
    // Pressure is stable if counter reaches some limit
    if (prevMotorState == 0){
      stateCount = millis() - startTime;
    }
    // Set back to zero if not
    else{
      stateCount = 0;
      startTime = millis();
    }
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
      TCCR1B &= (0 << CS11); // Turn off pulse stream
      break;
    case 1:
      //Move motor forwards at 2.5 mm/s
      TCNT1 = 0;
      OCR1A = OCR_2p5mmps;
      digitalWrite(directionPin, HIGH);
      TCCR1B |= (1 << CS11);  // Turn on motor
      //Serial.println("INCREASE PRESSURE");
      break;
    case 2:
      //Move motor back at 2.5 mm/s
      TCNT1 = 0;
      OCR1A = OCR_2p5mmps;
      digitalWrite(directionPin, LOW);
      TCCR1B |= (1 << CS11);  // Turn on motor
      //Serial.println("DECREASE PRESSURE");
      break;
    default:
      //Just in case nothing matches, stop timer/counter
      TCCR1B &= (0 << CS11); // Turn off pulse stream
      break;
  }
}


//External interrupt service function
//Set both step number and the counter used in for loop of stepping function to zero.
void forwardInterrupt() {
  TCCR1B &= (0 << CS11); // Turn off pulse stream
  //Stop other interrupts
  noInterrupts();
  extInterrupt = true;
  //Change setpoint to currrent pressure to prevent motion
  pressSetpoint = pressureAbs;
  stepCount -= limitSteps; //Keep track of volume change after moveMotor() call
  digitalWrite(directionPin, LOW);
  moveMotor();
  // Disable motor drivers
  digitalWrite(enablePin, HIGH);
  interrupts();
}


//External interrupt service Call function
//Set both step number, counter used for loop of stepping function to zero.
void backwardInterrupt() {
  TCCR1B &= (0 << CS11); // Turn off pulse stream
  //Stop other interrupts
  noInterrupts();
  extInterrupt = true;
  //Change setpoint to currrent pressure to prevent motion
  pressSetpoint = pressureAbs;
  stepCount += limitSteps; //Keep track of volume change after moveMotor() call
  digitalWrite(directionPin, HIGH);
  moveMotor();
  // Disable motor drivers
  digitalWrite(enablePin, HIGH);
  interrupts();
}


void moveMotor() {
  for (volatile unsigned long moveBackB = 0; moveBackB < limitSteps; moveBackB++) {
    //A step is executed when a rising edge is detected on Step motor input
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}


//Function to read input from python script and confirm pump position (TOP, LHS, RHS).
void handShake() {
  while (Serial.available() > 0) {
    shakeInput = Serial.readStringUntil('\n');
    if (shakeInput != ""){
      sprintf(data, "%s\n", shakeKey);
      Serial.write(data);
      if (shakeInput == shakeKey){
        shakeFlag = true;
        // Enable the motor after handshaking
        digitalWrite(enablePin, LOW);
        shakeInput = "";
        // delay(500);
        // Start reading serial for step/position
        TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
        // Initialise the time variables
        timeAtStep = micros();
        flushInputBuffer = Serial.readStringUntil('\n');
      }
    }
  }
}


//Function to read input in serial monitor and set the new desired pressure.
void readWriteSerial() {
  if (Serial.available() > 0) {
    firstDigit = Serial.read();
    // Control code sends capital S to receive stepCount
    // Capital S in ASCII is 83, so check for that:
    if (firstDigit == 83) {
      stepRecv = Serial.readStringUntil('\n');
      if (stepRecv == "Closed"){
        // Disable the motor
        digitalWrite(enablePin, HIGH);
        //Send disable message
        writeSerial('D');
      }
      else{
        stepIn = stepRecv.toInt();
        if (stepIn > maxSteps){
          stepIn = maxSteps;
        }
        stepError = stepIn - stepCount;
        //Send stepCount
        writeSerial('S');
        if (abs(stepError) > 0){
          // If piston not at desired position,
          // work out timeStep so that piston reaches after 1/fSamp seconds
          tStep = floor(tSampu/abs(stepError));
          if (tStep < tStep2k){
            tStep = tStep2k; // Limit fStep to 2 kHz
          }
        }
        else{
          // Set tStep to large value so no steps are made.
          tStep = oneHour;
        }
      }
    }
    else {
      flushInputBuffer = Serial.readStringUntil('\n');
    }
  }
}


void stepAftertStep(){
  // Step the motor if enough time has passed.
  timeNow = micros();
  timeSinceStep = timeNow - timeAtStep;
  if (tStep == oneHour){
    ; // Do nothing
  }
  else if (timeSinceStep >= tStep){
    if (stepError > 0){
      if (stepCount < maxSteps){
        digitalWrite(directionPin, HIGH);
        digitalWrite(stepPin, HIGH);
        digitalWrite(stepPin, LOW);
        stepCount += 1;
      }
    }
    else if (stepError < 0){
      if (stepCount > 0){
        digitalWrite(directionPin, LOW);
        digitalWrite(stepPin, HIGH);
        digitalWrite(stepPin, LOW);
        stepCount -= 1;
      }
    }
    timeAtStep = micros();
    stepError = stepIn - stepCount;
  }
}


void writeSerial(char msg){
  writeTime = millis();
  if (msg == 'S'){ // Normal operation, send stepCount etc
    sprintf(data, "%04d,%d,%lu%s", stepCount, int(pressureAbs*10), writeTime, endByte);
  }
  else if (msg == 'D'){ // Python cut off comms, acknowledge this
    sprintf(data, "%s%s,%d,%lu%s", disableMsg, shakeKey, int(pressureAbs*10), writeTime, endByte);
  }
  else if (msg == 'L'){ // Limit switch hit, advise Python
    sprintf(data, "%s%s,%d,%lu%s", limitHit, shakeKey, int(pressureAbs*10), writeTime, endByte);
  }
  else if (msg == 'p'){ // Calibrating
    sprintf(data, "%04d%s,%d,%lu%s", stableTime-stateCount, shakeKey, int(pressureAbs*10), writeTime, endByte);
  }
  else if(msg = 'P'){ // Calibration finished
    sprintf(data, "%04d%s,%d,%lu%s", stepCount, shakeKey, int(pressureAbs*10), writeTime, endByte);
  }
  Serial.write(data);
}


void loop() {
  if (extInterrupt == true){
    pumpState = 0;//Limits hit
  }
  else if(shakeFlag == false){
    pumpState = 1;//Handshake
  }
  else if(pressFlag == false){//CHANGE TO FALSE TO ACTIVATE
    pumpState = 2;//Calibration
  }
  else if(!Serial){
    pumpState = 3;//Disconnection
  }
  else{
    pumpState = 4;//Active
  }

  switch(pumpState){
    //////////////////////////////////////////////////////////////////////////////////////////
    //Limits hit
    case 0:
      //Make sure motor is disabled 
      digitalWrite(enablePin, HIGH);
      // Turn off timers for interrupts
      // TCCR2B = 0;
      // Turn off calibration pulse stream
      TCCR1B &= (0 << CS11);  
      stateCount = 0;
      startTime = millis();
      if (sampFlag == true) {
        pressureProtect();
        sampFlag = false;
      }
      flushInputBuffer = Serial.readStringUntil('\n');
      // Notify that limit hit
      writeSerial('L');
      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    //Handshake/disconnection
    case 1:
      handShake();
      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    //Calibration
    // HANDLE DISCONNECTION AFTER CALIBRATION HAS BEGUN
    case 2:
      if (sampFlag == true) {
        pressInitZeroVol();
        // If enough time has passed, say volume is 0, tell python and move on
        if (stateCount >= stableTime){
          // Step count should now be zero - muscle empty.
          stepCount = 0;
          // Notify that calibration is done
          writeSerial('P');
          TCCR1B &= (0 << CS11); // Turn off pulse stream
          pressFlag = true;
        }
        else{
          // Say calibration in progress
          writeSerial('p');
        }
        sampFlag = false;
      }
      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    //Disconnection
    case 3:
      digitalWrite(enablePin, HIGH);
      // Turn off timers for interrupts
      TCCR2B = 0;
      // Turn off calibration pulse stream
      TCCR1B &= (0 << CS11); 
      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    //Active
    case 4:
      // Call overpressure protection every 6th Timer2 interrupt
      if (sampFlag == true) {
        pressureProtect();
        sampFlag = false;
      }
      if (serFlag == true){
        readWriteSerial(); // Read stepIn, send stepCount and pressure
        serFlag = false;
      }
      // Step the motor if enough time has passed.
      stepAftertStep();
      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    // Unrecognised state, act as if limit hit
    default:
      //Make sure motor is disabled 
      digitalWrite(enablePin, HIGH);
      // Turn off timers for interrupts
      TCCR2B = 0;
      // Turn off calibration pulse stream
      TCCR1B &= (0 << CS11);  
      stateCount = 0;
      startTime = millis();
      pressureProtect();
      flushInputBuffer = Serial.readStringUntil('\n');
      // Notify Python
      writeSerial('L');
      break;
  }
}
