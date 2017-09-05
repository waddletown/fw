/*
Odometer.h
Source: https://github.com/DrGFreeman/RasPiBot202V2

MIT License

Copyright (c) 2017 Julien de la Bruere-Terreault <drgfreeman@tuta.io>

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
An odometer class for a differential drive robot
*/

#ifndef Odometer_h
#define Odometer_h

#include <Arduino.h>

class Odometer
{
public:
  /* Constructor
    meters_per_tick: the distance (in meters) travelled for one encoder tick count.
    wheelbase: the distance between the wheels in meters
  */
  Odometer(float meters_per_tick, float wheelbase);

  // Return the angular velocity in rad/s
  float getOmega();

  // Return the theta angle
  float getTheta();

  // Return the speed in distance unit / s
  float getSpeed();

  // Return the left speed in distance unit / s
  float getSpeedLeft();

  // Return the right speed in distance unit / s
  float getSpeedRight();

  // Return the time step between last two update calls in micro seconds
  unsigned int getTimeStep();

  // Return the X position
  float getX();

  // Return the Y position
  float getY();

  // Reset the odometer
  void reset();

  // Update odometer status
  void update(int countLeft, int countRight);

private:
  // Position variables
  float _x;
  float _y;
  float _theta;

  // Speed variables
  float _speedLeft;
  float _speedRight;
  float _omega;

  // Encoder counts variables
  int _lastCountLeft;
  int _lastCountRight;

  // Geometrical attributes
  float _meters_per_tick;
  float _wheelbase;

  // Time step variables
  unsigned long _lastUpdateTime;
  unsigned int _timeStep;
};

#endif
