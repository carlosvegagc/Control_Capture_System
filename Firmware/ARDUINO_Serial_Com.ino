
#define USE_TIMER_1 true

//#include "TimerInterrupt.h"

#include "serial_functions.h"
#include "bsp_functions.h"

#include <ReducedMPU9250.h>
#include <Servo.h>
#include <Stepper.h>



#define TIMER1_INTERVAL_MS 10000
#define WAIT_TIME_SERVO 100

//DEFINITION OF OBJECTS
ReducedMPU9250 mpu(false);

//EVENT FLAGS
bool f_serialNewLine = false; // whether the string is complete

//GLOBAL VARIABLES
String inputString = ""; // a String to hold incoming data
unsigned int outputPin1 = LED_BUILTIN;
Servo miServo; // Creo objeto de la clase Servo.

//SERVO PINOUT
#define SERVO_PIN 9

//STEPPER PINOUT
const int dirPin = 8;
const int stepPin = 6;
const int enable = 7;
float PosIni = 0;

//MULTIPLEXER PINOUT
const int muxSelector = 4; // A activo a nivel bajo, B a nivel alto
const int muxEnable = 5;   // G activo a nivel bajo (enable)

//Switch Homming Pin
#define interrupPin 2

//Function for analysing the system periodically and sending information the host
void TimerHandler1(unsigned int outputPin = LED_BUILTIN)
{
  static bool toggle1 = false;
  static bool started = false;

  if (!started)
  {
    started = true;
    pinMode(outputPin, OUTPUT);
  }

  Serial.print("OK\n");

  digitalWrite(outputPin, toggle1);
  toggle1 = !toggle1;
}

void setup()
{
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  /*
  Serial.print(F("\nStarting Antenna Development Platform "));
  Serial.println(BOARD_TYPE);
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));

 
  ITimer1.init();

  if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, TimerHandler1, outputPin1))
  {
    Serial.print(F("Starting  ITimer1 OK, millis() = ")); Serial.println(millis());
  }
  */

  //Inits the MPU
  if (!init_IMU(&mpu))
    Serial.println("ERROR: NOT ABLE TO INIT THE IMU");

  //Inits the servo
  miServo.attach(SERVO_PIN);

  // Inits the stepper pins
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enable, OUTPUT);

  // Init the multiplexer  pins
  pinMode(muxSelector, OUTPUT);
  pinMode(muxEnable, OUTPUT);
  digitalWrite(muxEnable, LOW);   // Enable activo.
  digitalWrite(muxSelector, LOW); // Habilitamos comunicaciones con A (Arduino)

  //Initi the switch pin for homming
  pinMode(interrupPin, INPUT);
}

int exeCommand(SerialCommand inCommand)
{

  int res = 0;

#ifdef DEBUG
  Serial.print(F("Recieved Command "));
  Serial.print(inCommand.command);
  Serial.print(F(" with parameters = "));
  Serial.println(inCommand.nParams);

  for (short i = 0; i < inCommand.nParams; i++)
  {
    Serial.println(inCommand.params[i]);
  }
#endif

  int idCommand = inCommand.command;

  //Get Acceleration and Magnetometer values
  if (inCommand.command == 'G')
  {

    float AccG[] = {0.0, 0.0, 0.0};
    float MagCal[] = {0.0, 0.0, 0.0};
    ValoresAccyMag(&mpu, AccG, MagCal);

    return 0;
  }
  // Calibrate magnetometer
  else if (inCommand.command == 'C')
  {
    mpu.CalibrateMagnetometer();
    Serial.println("C:OK");
    return 0;
  }

  // Introduce offset to the acelerometer
  else if (inCommand.command == 'U')
  {
    if (inCommand.nParams != 3)
    {
      return -2;
    }
    int mode = inCommand.params[0].toInt();
    int offset = inCommand.params[1].toInt();
    int axis = inCommand.params[1].toInt();
    if (mode == 0)
    {
      updateAccelerometerCalibrationScale(&mpu, offset, axis);
      Serial.println("Update scale");
    }
    else if (mode == 1)
    {
      updateAccelerometerCalibrationOffsets(&mpu, offset, axis);
      Serial.println("Update offset");
    }
    else
    {
      return -2;
    }
    return 0;
  }

  // Moving the servo to an specified position
  else if (inCommand.command == 'S')
  {
    if (inCommand.nParams != 1)
    {
      return -2;
    }
    float pos = inCommand.params[0].toFloat();

    MovServoMotor(&miServo, pos, WAIT_TIME_SERVO);

    //Returns the the servo has arrived
    Serial.println("S:OK");
    return 0;
  }

  // Moving the stepper to an specified position
  else if (inCommand.command == 'M')
  {
    if (inCommand.nParams != 1)
    {
      return -2;
    }
    float pos = inCommand.params[0].toFloat();

    MovStepperMotor(dirPin, stepPin, enable, pos, &PosIni);

    //Returns the the servo has arrived
    Serial.println("M:OK");
    return 0;
  }

  // Switching the MUX for communicating with the GPS
  else if (inCommand.command == 'A')
  {
    if (inCommand.nParams != 1)
    {
      return -2;
    }
    float switchTime = inCommand.params[0].toFloat();

    switchUSART(muxSelector, muxEnable, switchTime);

    //Clears the buffers 
    while(Serial.available() > 0) {
      char t = Serial.read();
    }

    //Returns the the servo has arrived
    Serial.println("A:OK");
    return 0;
  }

  // Function for homming
  else if (inCommand.command == 'H')
  {
    float pos = 180;
    MovServoMotor(&miServo, pos, WAIT_TIME_SERVO);
    delay(200);
    while((digitalRead(interrupPin) == LOW )&& (pos > 0)){
      //Serial.println(pos);
      MovServoMotor(&miServo, pos, WAIT_TIME_SERVO);
      pos = pos - 10;
    }
    
    //Returns the ACK
    Serial.println("H:OK");
    return 0;
    
  }

  else
  {
    return -1;
  }

  return res;
}

void loop()
{
  // put your main code here, to run repeatedly:
  // print the string when a newline arrives:
  if (f_serialNewLine)
  {
    SerialCommand inCommand = decodeSerialCommand(&inputString);

    if (exeCommand(inCommand) < 0)
    {
      Serial.println("E:0");
      Serial.println("ERROR, Unknown command");
    }

    //Serial.println("Command Executed");

    // clear the string:
    inputString = "";
    f_serialNewLine = false;
  }
}

/* 
  Serial event that saves the incoming data into a string until it recieves a \n
*/
void serialEvent()
{
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();

    // add it to the inputString:
    inputString += inChar;

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
    {
      f_serialNewLine = true;
    }
  }
}
