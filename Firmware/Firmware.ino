
/**
*    Fw to control the triger communications for the HSI Capture System.
*
*    Requires the following packages: Bolder_Flight_Systems_MPU9250, Bolder_Flight_Systems_Eigen, Bolder_Flight_Systems_Unit_Conversions
*
*    Authors: Carlos Vega, 
*    Email: cvega@iuma.ulpgc.es
*/


#define USE_TIMER_1 true
//#define DEBUG


#include "serial_functions.h"
#include "bsp_functions.h"
#include "mpu9250.h"
#include <Ticker.h>
#include <Wire.h>

#define TIMER1_INTERVAL_MS 10000
#define WAIT_TIME_SERVO 100

Ticker timer;


bool getMean;
#define BUFFER_SIZE 10
double IMUBuffer[BUFFER_SIZE];
#define SDA_PIN 14
#define SCL_PIN 27

/* Mpu9250 object, SPI bus, CS on pin 10 */
//bfs::Mpu9250 imu(&SPI, 10);
bfs::Mpu9250 imu(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

//EVENT FLAGS
bool f_serialNewLine = false; // whether the string is complete

//GLOBAL VARIABLES
String inputString = ""; // a String to hold incoming data

//Button variables.
const int buttonPin = 34;  // GPIO pin connected to the button
bool buttonState = false;     // variable to store the button state

// LED Variables
const int ledPin = 20;  // GPIO pin connected to the LED
const int ledMotionPin = 26;  // GPIO pin connected to the LED



void timerTick()
{
  digitalWrite(ledPin, !digitalRead(ledPin));  // toggle the LED state
}

/* 
  Serial event that saves the incoming data into a string until it recieves a \n
*/
void onReceiveFunction()
{
  #ifdef DEBUG
  Serial.println("Event Serial");
  #endif

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


// Code for the initialization of the system
void setup()
{
  // initialize serial:
  Serial.begin(115200);
  Serial.onReceive(onReceiveFunction); 

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  
  Serial.print(F("\nStarting Antenna Development Platform "));
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));

  //Inits the MPU
 /* Start the SPI bus */
  //SPI.begin();
  /* Initialize and configure IMU */
  Wire.begin(SDA_PIN,SCL_PIN);
  Serial.println("Initializing IMU...");
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
  }
  else {
    Serial.println("IMU connected correctly");
  }

  //Init the button.
  pinMode(ledPin, OUTPUT);
  pinMode(ledMotionPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPressed, RISING);
  // attachInterrupt(digitalPinToInterrupt(buttonPin), buttonReleased, FALLING);
  
  timer.attach(10, timerTick);  // call the toggleLED function every 10 milliseconds
  
  
  
  
  
  getMean = false;
  for (int i=0; i<BUFFER_SIZE; i++){
    IMUBuffer[i] = NAN;
  }

  
  
  
}

// Function to handle the communication commands.
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

    // Read one value from the IMU
    if (imu.Read()) {

      Serial.print("Aceleration on X: ");
      Serial.println(imu.accel_x_mps2());
      Serial.print("Aceleration on Y: ");
      Serial.println(imu.accel_y_mps2());
      Serial.print("Aceleration on Z: ");
      Serial.println(imu.accel_z_mps2());
      
//      Serial.print(imu.new_imu_data());
//      Serial.print("\t");
//      Serial.print(imu.new_mag_data());
//      Serial.print("\t");
//      Serial.print(imu.accel_x_mps2());
//      Serial.print("\t");
//      Serial.print(imu.accel_y_mps2());
//      Serial.print("\t");
//      Serial.print(imu.accel_z_mps2());
//      Serial.print("\t");
//      Serial.print(imu.gyro_x_radps());
//      Serial.print("\t");
//      Serial.print(imu.gyro_y_radps());
//      Serial.print("\t");
//      Serial.print(imu.gyro_z_radps());
//      Serial.print("\t");
//      Serial.print(imu.mag_x_ut());
//      Serial.print("\t");
//      Serial.print(imu.mag_y_ut());
//      Serial.print("\t");
//      Serial.print(imu.mag_z_ut());
//      Serial.print("\t");
//      Serial.println(imu.die_temp_c());
    }

    return 0;
  }

  // Starts the capture
  else if (inCommand.command == 'S')
  {

    digitalWrite(ledPin, HIGH);  // turn on the LED

    return 0;
  }

  
  // Ends the capture
  else if (inCommand.command == 'E')
  {
    
    digitalWrite(ledPin, LOW);   // turn off the LED

    return 0;
  }

  // Version Command
  else if (inCommand.command == 'V')
  {
    
    Serial.println("V:0.1 Control Capture System");  
    return 0;

  }

  else if (inCommand.command == 'I')
  {
    digitalWrite(ledMotionPin, !digitalRead(ledMotionPin));
    getMean = !getMean;
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

  // This code is executed when a new command arrives
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

  // Read if there bottom is pressed.
  int newButtonState = digitalRead(buttonPin);  // read the button state
  // Checks for the rising event
  if (newButtonState == HIGH && buttonState == LOW) {
    Serial.println("P:");
  }
  buttonState = newButtonState;

  if (getMean){
    Serial.println(sizeof(IMUBuffer)/sizeof(double));
    
    for (int i=0; i<=BUFFER_SIZE; i++){
      if (isnan(IMUBuffer[i])){
        Serial.println("...");
        IMUBuffer[i]=i; // read IMU
        Serial.println(IMUBuffer[i]);
        continue;
      }
    }
    getMean=false;
  }

  
  // Wait for 5 seconds
  delay(5);
  
}

double moveHeader(double bufferArray){
  for (int i=1; i<= bufferArray; i++){
    
  }
}
