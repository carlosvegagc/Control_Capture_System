
#include "bsp_functions.h"

/**************************************
 * IMU FUNCTIONS
 *************************************/

bool init_IMU(ReducedMPU9250 *accelMag)
{
  Wire.begin();
  accelMag->initialize();
  accelMag->setFullScaleAccelRange(MPU9250_ACCEL_FS_2);

  return accelMag->testConnection();
}

void ValoresAccyMag(ReducedMPU9250 *accelMag, float *AccG, float *MagCal)
{
  bool measureOk;

  accelMag->getLinearAccelerationG(&AccG[0], &AccG[1], &AccG[2], true);
  accelMag->getMagnetometerCalibratedData(&MagCal[0], &MagCal[1], &MagCal[2], &measureOk);
  //accelMag.getMagnetometerReading(&Mag[0], &Mag[1], &Mag[2], &measureOk, true);

  // Imprimir por pantalla (comprobación).

  Serial.print("G:");
  Serial.print(AccG[0]);
  Serial.print(" ");
  Serial.print(AccG[1]);
  Serial.print(" ");
  Serial.print(AccG[2]);
  Serial.print(" ");
  Serial.print(MagCal[0]);
  Serial.print(" ");
  Serial.print(MagCal[1]);
  Serial.print(" ");
  Serial.print(MagCal[2]);
  Serial.println();
  delay(1000);
}

void updateAccelerometerCalibrationOffsets(ReducedMPU9250 *accelMag, int axisIndex, int increment)
{
  accelMag->UpdateAccelerometerOffset(axisIndex, increment);
}

void updateAccelerometerCalibrationScale(ReducedMPU9250 *accelMag, int axisIndex, int increment)
{
  accelMag->UpdateAccelerometerScale(axisIndex, increment);
}

/**************************************
 * SERVO FUNCTIONS
 *************************************/

void MovServoMotor(Servo *miServo, float posFinal, int tiempoEspera,int offset)
{
  //posFinal = map(posFinal, 0, 1023, 0, 180); // scale it to use it with the servo (value between 0 and 180)
  miServo->write((int)((180 - posFinal) + offset)
  
  );
  delay(tiempoEspera);
}

/**************************************
 * STEPPER FUNCTIONS
 *************************************/

void MovStepperMotor(const int dirPin, const int stepPin, const int enable, float PosFinal, float *PosIni)
{
  int stepDelay = 250;
  digitalWrite(enable, LOW); // Enable.

  if (*PosIni > PosFinal)
  {
    digitalWrite(dirPin, LOW); // --> COMPROBAR SI ES EL LADO CORRECTO
    if ((*PosIni - PosFinal) >= movMax)
    {
      for (int x = 0; x < (movMax / gradosPorPaso); x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
      }
      *PosIni = *PosIni - movMax;
    }
    else
    {
      for (int x = 0; x < ((*PosIni - PosFinal) / gradosPorPaso); x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
      }
      *PosIni = PosFinal;
    }
  }
  else if (*PosIni < PosFinal)
  {
    digitalWrite(dirPin, HIGH); // --> COMPROBAR SI ES EL LADO CORRECTO
    if ((PosFinal - *PosIni) >= movMax)
    {
      for (int x = 0; x < (movMax / gradosPorPaso); x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
      }
      *PosIni = *PosIni + movMax;
    }
    else
    {
      for (int x = 0; x < ((PosFinal - *PosIni) / gradosPorPaso); x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
      }
      *PosIni = PosFinal;
    }
  }
  else
  {
    *PosIni = PosFinal;
  }
}

/**************************************
 * MULTIPLEXER FUNCTIONS
 *************************************/
void switchUSART(const int selector, const int enable, int tiempo)
{
  digitalWrite(selector, HIGH); // Habilitamos comunicaciones con B (GPS)
  delay(tiempo);                // Comunicamos durante el tiempo especificado (ms)
  digitalWrite(selector, LOW);  // Habilitamos comunicaciones con A (Arduino)
}
