# waddletown fw
This is the place where we have firmware for the Waddle Town robot.

## Struct Reference

```
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
```

| Variable Name     | Type     | Address |
| ----------------- | -------  | ------- |
| yellow            | bool     | 0       |
| green             | bool     | 1       |
| red               | bool     | 2       |
| buttonA           | bool     | 3       |
| buttonB           | bool     | 4       |
| buttonC           | bool     | 5       |
| VxCommand         | float    | 6-9     |
| VthetaCommand     | float    | 10-13   |
| Vx                | float    | 14-17   |
| Vtheta            | float    | 18-21   |
| x                 | float    | 22-25   |
| y                 | float    | 26-29   |
| theta             | float    | 30-33   |
| resetOdometer     | bool     | 34      |
| batteryMillivolts | uint16_t | 35-36   |
| playNotes         | bool     | 37      |
| notes             | char[14] | 38-51   |
