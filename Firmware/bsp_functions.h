
#ifndef BSP_F
#define BSP_F

#include <Wire.h>
#include <ReducedMPU9250.h>
#include <CalculateAngles.h>

#include <Servo.h>
#include <Stepper.h>

#define steps 200
#define reduccion 3.6
#define microPasos 16
#define vueltaAntena (steps * microPasos * reduccion)
#define gradosPorPaso (360 / vueltaAntena)
#define movMax 5

//ReducedMPU9250 accelMag;

bool init_IMU(ReducedMPU9250 *accelMag);
void ValoresAccyMag(ReducedMPU9250 *accelMag, float *AccG, float *MagCal);
void updateAccelerometerCalibrationOffsets(ReducedMPU9250 *accelMag, int axisIndex, int increment);
void updateAccelerometerCalibrationScale(ReducedMPU9250 *accelMag, int axisIndex, int increment);
void MovServoMotor(Servo *miServo, float posFinal, int tiempoEspera, int offset);
void MovStepperMotor(const int dirPin, const int stepPin, const int enable, float PosFinal, float *PosIni);
void switchUSART(const int selector, const int enable, int tiempo);
#endif