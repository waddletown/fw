/*
Odometer.cpp
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

#include <Odometer.h>
#include <math.h>

// Constructor
Odometer::Odometer(float meters_per_tick, float wheelbase)
{
  // Set meters_per_tick and wheelbase properties
  _meters_per_tick = meters_per_tick;
  _wheelbase = wheelbase;

  // Initialize variables
  _lastCountLeft = 0;
  _lastCountRight = 0;
  reset();
}

// Return the angular velocity in rad/s
float Odometer::getOmega()
{
  return _omega;
}

// Return the theta angle
float Odometer::getTheta()
{
  return _theta;
}

// Return the speed in distance unit / s
float Odometer::getSpeed()
{
  return (_speedLeft + _speedRight) / 2;
}

// Return the left speed in distance unit / s
float Odometer::getSpeedLeft()
{
  return _speedLeft;
}

// Return the right speed in distance unit / s
float Odometer::getSpeedRight()
{
  return _speedRight;
}

// Return the time step between last two update calls in micro seconds
unsigned int Odometer::getTimeStep()
{
  return _timeStep;
}

// Return the X position
float Odometer::getX()
{
  return _x;
}

// Return the Y position
float Odometer::getY()
{
  return _y;
}

// Reset the Odometer
void Odometer::reset()
{
  _x = 0;
  _y = 0;
  _theta = 0;

  _speedLeft = 0;
  _speedRight = 0;
  _omega = 0;

  _lastUpdateTime = micros();
}

// Update position and speed with new encoder counts
void Odometer::update(int countLeft, int countRight)
{
  // Calculate time step since last update
  unsigned long currentTime = micros();
  _timeStep = currentTime - _lastUpdateTime;

  // Store current time for next update
  _lastUpdateTime = currentTime;

  // Calculate difference in counts since last update
  int diffLeft = countLeft - _lastCountLeft;
  int diffRight = countRight - _lastCountRight;

  // Store current counts for next update
  _lastCountLeft = countLeft;
  _lastCountRight = countRight;

  // Calculate displacement
  float distLeft = diffLeft * _meters_per_tick;
  float distRight = diffRight * _meters_per_tick;
  float distCenter = (distLeft + distRight) / 2;
  float deltaTheta = (distRight - distLeft) / _wheelbase;

  // Calculate new Position
  _x += distCenter * cos(_theta);
  _y += distCenter * sin(_theta);
  _theta += deltaTheta;

  // Keep theta within 0 & 2 * Pi
  if (_theta >= 2 * M_PI)
  {
    _theta -= 2 * M_PI;
  }
  else if (_theta < 0)
  {
    _theta += 2 * M_PI;
  }

  // Calculate speed
  _speedLeft = distLeft / float(_timeStep / 1E6);
  _speedRight = distRight / float(_timeStep / 1E6);
  _omega = deltaTheta / float(_timeStep / 1E6);;
}
