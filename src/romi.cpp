/*
waddle_fw.cpp
Source: https://github.com/waddletown/fw

MIT License

Copyright (c) 2017 Bradley Powers <brad@lowellmakes.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
Romi Raspberry Pi Slave
This is the main sketch to run on the Romi Robot Controller for the
Waddle Town robot.
*/
#include <Arduino.h>
#include <PID.h>
#include <Romi32U4.h>
#include <PololuRPiSlave.h>
#include <Odometer.h>
#include <PID.h>

// Motor Encoder Parameters and Robot Geometry
// Wheel diameter in meters
const float wheel_diameter = 0.068;

// Encoder ticks per output revolution
const float ticks_per_output_revolution = 12*120;

// Distance travelled (in meters) per encoder tick
const float meters_per_tick = (M_PI * wheel_diameter)/ticks_per_output_revolution;

// Wheelbase in meters
const float wheelbase = 0.148;

// Define motor PID gains
const float Kp = 2000.0;
const float Ki = 2000.0;
const float Kd = 0.0;

// Define motors max command
const float motorsMaxCommand = 400;

// Define speed variables for acceleration control
int lastSpeedCmdLeft = 0;
int lastSpeedCmdRight = 0;

// Define maximum speed command change per time step
const int accelMax = 1000;

// Heartbeat, to be displayed on LED.
int heartbeatCount = 0;
bool heartbeat = false;

// Odometer
Odometer odometer(meters_per_tick, wheelbase);
// Motor PID controllers
PID pidLeft(Kp, Ki, Kd);
PID pidRight(Kp, Ki, Kd);

// Define objects from Romi32u4 Library
// Motors
Romi32U4Motors motors;
// Encoders
Romi32U4Encoders encoders;
// Buttons
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
// Buzzer (TODO: DELETE ME?)
PololuBuzzer buzzer;

struct Data
{
  bool yellow, green, red;
  bool buttonA, buttonB, buttonC;

  float VxCommand, VthetaCommand;
  float Vx, Vtheta;
  float x, y, theta;

  bool resetOdometer;

  uint16_t batteryMillivolts;

  bool playNotes;
  char notes[14];
};

PololuRPiSlave<struct Data,10> slave;

// Sets the motor speeds using PID controllers
void setMotorSpeeds(float speedLeft, float speedRight)
{
  // get speed command from PID controllers
  int speedCmdLeft = pidLeft.getCmdAutoStep(speedLeft, odometer.getSpeedLeft());
  int speedCmdRight = pidRight.getCmdAutoStep(speedRight, odometer.getSpeedRight());

  // Handle speed commands

  // Control maximum acceleration
  if (speedCmdLeft - lastSpeedCmdLeft > accelMax)
  {
    speedCmdLeft = lastSpeedCmdLeft + accelMax;
  }
  if (speedCmdLeft - lastSpeedCmdLeft < -accelMax)
  {
    speedCmdLeft = lastSpeedCmdLeft - accelMax;
  }
  if (speedCmdRight - lastSpeedCmdRight > accelMax)
  {
    speedCmdRight = lastSpeedCmdRight + accelMax;
  }
  if (speedCmdRight - lastSpeedCmdRight < -accelMax)
  {
    speedCmdRight = lastSpeedCmdRight - accelMax;
  }

  // Stop immediately if target speed is zero
  if (speedLeft == 0)
  {
    speedCmdLeft = 0;
    pidLeft.reset();
  }
  if (speedRight == 0)
  {
    speedCmdRight = 0;
    pidRight.reset();
  }

  // Set motor speeds
  motors.setSpeeds(speedCmdLeft, speedCmdRight);

  lastSpeedCmdLeft = speedCmdLeft;
  lastSpeedCmdRight = speedCmdRight;
}

void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(20);

  // Play startup sound.
  buzzer.play("v10>>g16>>>c16");
  delay(200);

  // Set PID controllers command range
  pidLeft.setCmdRange(-motorsMaxCommand, motorsMaxCommand);
  pidRight.setCmdRange(-motorsMaxCommand, motorsMaxCommand);
}

void loop()
{
  // Get current time
  unsigned long currentTime = micros();

  // Read encoders
  int countsLeft = encoders.getCountsLeft();
  int countsRight = encoders.getCountsRight();

  // Update odometer
  odometer.update(countsLeft, countsRight);

  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  slave.updateBuffer();

  // Write various values into the data structure.
  slave.buffer.buttonA = buttonA.isPressed();
  slave.buffer.buttonB = buttonB.isPressed();
  slave.buffer.buttonC = buttonC.isPressed();

  // Change this to readBatteryMillivoltsLV() for the LV model.
  slave.buffer.batteryMillivolts = readBatteryMillivolts();


  // Read reset odometer flag and reset odometer values if true
  if (slave.buffer.resetOdometer)
  {
    odometer.reset();
    slave.buffer.resetOdometer = 0;
  }
  // Write odometry values into data structure
  slave.buffer.x = odometer.getX();
  slave.buffer.y = odometer.getY();
  slave.buffer.theta = odometer.getTheta();
  slave.buffer.Vx = odometer.getSpeed();
  slave.buffer.Vtheta = odometer.getOmega();

  // Read LED values from buffer and set LED states
  ledYellow(slave.buffer.yellow);
  ledGreen(slave.buffer.green);
  // ledRed(slave.buffer.red);

  heartbeatCount++;
  if (heartbeatCount >= 50)
  {
    heartbeatCount = 0;
    heartbeat = !heartbeat;
    ledRed(heartbeat);
  }


  // Read speed and turn rate from buffer and calculate motor speeds
  float VxCommand = slave.buffer.VxCommand;
  float VthetaCommand = slave.buffer.VthetaCommand;
  float leftSpeed  = VxCommand - VthetaCommand * wheelbase / 2;
  float rightSpeed = VxCommand + VthetaCommand * wheelbase / 2;

  setMotorSpeeds(leftSpeed, rightSpeed);

  // Playing music involves both reading and writing, since we only
  // want to do it once.
  static bool startedPlaying = false;

  if(slave.buffer.playNotes && !startedPlaying)
  {
    buzzer.play(slave.buffer.notes);
    startedPlaying = true;
  }
  else if (startedPlaying && !buzzer.isPlaying())
  {
    slave.buffer.playNotes = false;
    startedPlaying = false;
  }

  // We are done writing; make modified data availale to I2C master
  slave.finalizeWrites();

  // Ensure a constant time step of the main loop (10 milliseconds)
  while (micros() - currentTime < 10000)
  {
    // Do nothing; wait until time step is reached
  }
}
