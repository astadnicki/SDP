/*
Odometry 2022-2023 WIP
John Boesen
Collection of Functions to Drive Robot Motors/Acutators

PIND 2 - Forward Rear Left
PIND 3 - Forward Front Left
PIND 4 - Forward Rear Right
PIND 5 - Forward Front Right

PINB 0 - Reverse Rear Left
PINB 1 - Reverse Front Left
PINB 2 - Reverse Rear Right
PINB 3 - Reverse Front Right

Test Order:
-readIMU() in readIMU.cpp
-xforward()
-xdriveRight()/driveLeft()
-xdigCycle()

TODO
- IMU code/test

*/
//#include "movement.h"
#define RESOLUTION 5
#define __DELAY_BACKWARD_COMPATIBLE__

void delay_us_10us(int ms)  // variable compatible delay wrapper
{
  while (0 < ms) {
    _delay_us(1);
    ms -= 1;
  }
}

int driveForward(float speed) {

  //Send forward PWM output to All Wheels till on grid space
  if (speed > 0) {
    int pulse;
    pulse = speed * 10;
    int i = 0;
    while (i < 1000) {  //Drive w/o IMU data (Only use for tele-op)
      PORTD |= (1 << PORT2);
      PORTD |= (1 << PORT3);
      PORTD |= (1 << PORT4);
      PORTD |= (1 << PORT5);
      PORTB &= ~(1 << PORT0);
      PORTB &= ~(1 << PORT1);
      PORTB &= ~(1 << PORT2);
      PORTB &= ~(1 << PORT3);
      delay_us_10us(pulse);
      PORTD &= ~(1 << PORT2);
      PORTD &= ~(1 << PORT3);
      PORTD &= ~(1 << PORT4);
      PORTD &= ~(1 << PORT5);
      delay_us_10us(10 - pulse);
      i++;
    }

    return 0;
  } else {
    int pulse;
    pulse = -1 * speed * 10;
    int i = 0;
    while (i < 1000) {  //Drive w/o IMU data (Only use for tele-op)
      PORTB |= (1 << PORT0);
      PORTB |= (1 << PORT1);
      PORTB |= (1 << PORT2);
      PORTB |= (1 << PORT3);
      PORTD &= ~(1 << PORT2);
      PORTD &= ~(1 << PORT3);
      PORTD &= ~(1 << PORT4);
      PORTD &= ~(1 << PORT5);
      delay_us_10us(pulse);
      PORTB &= ~(1 << PORT0);
      PORTB &= ~(1 << PORT1);
      PORTB &= ~(1 << PORT2);
      PORTB &= ~(1 << PORT3);
      delay_us_10us(10 - pulse);
      i++;
    }

    return 0;
  }
}

int driveRight(float ang, float speed) {

  // if (ang>3.14 || ang<0){
  //   return 1;
  // }
  // if(speed>1){
  //   return 2;
  // } else if(speed<0){
  //   return 3;
  // }

  float angle;

  //Setup pulse and angle variables
  int pulse = speed * 10;
  //angle = readIMU(0);

  //while(angle<ang){
  int i = 0;  //Drive w/o IMU data (Only use for tele-op)
  while (i < 1000) {
    //Send forward PWM output to Left Wheels until IMU reads correct angle
    PORTD |= (1 << PORT2);
    PORTD |= (1 << PORT3);
    PORTB |= (1 << PORT2);
    PORTB |= (1 << PORT3);
    PORTB &= ~(1 << PORT0);
    PORTB &= ~(1 << PORT1);
    PORTD &= ~(1 << PORT4);
    PORTD &= ~(1 << PORT5);
    delay_us_10us(pulse);
    PORTD &= ~(1 << PORT2);
    PORTD &= ~(1 << PORT3);
    PORTB &= ~(1 << PORT2);
    PORTB &= ~(1 << PORT3);
    delay_us_10us(10 - pulse);

    i++;
  }
  //Send reverse PWM output to Right Wheels until IMU reads correct angl

  //Grab IMU data to check angle
  //angle = readIMU(0);
  //}

  //Send forward PWM output to All Wheels till on grid space
  //if(driveForward(speed)!=0){
  //return 5;
  //}

  return 0;
}

int driveLeft(float deg, float speed) {
  // if (deg>3.14 || deg<0){
  //     return 1;
  // }
  // if(speed>1){
  //     return 2;
  // } else if(speed<0){
  //     return 3;
  // }

  float angles[3];

  //Setup pulse and angle variables
  int pulse = (speed)*10;
  //angle = readIMU(0);

  /*while(angle<deg){

        //Send forward PWM output to Left Wheels until IMU reads correct angle
        PORTD |= (1<<PORT2);
        PORTD |= (1<<PORT3);
        _delay_us(fpulse);
        PORTD &= ~(1<<PORT2);
        PORTD &= ~(1<<PORT3);
        _delay_us(2000-fpulse);

        //Send reverse PWM output to Right Wheels until IMU reads correct angle
        PORTD |= (1<<PORT4);
        PORTD |= (1<<PORT5);
        _delay_us(rpulse);
        PORTD &= ~(1<<PORT4);
        PORTD &= ~(1<<PORT5);
        _delay_us(2000-rpulse);

        //Grab IMU data to check angle
        angle = readIMU(0);
        delay_us(fpulse+rpulse);
    }
    */
  int i = 0;  //Drive w/o IMU data (Only use for tele-op)
  while (i < 1000) {
    PORTD |= (1 << PORT4);
    PORTD |= (1 << PORT5);
    PORTB |= (1 << PORT0);
    PORTB |= (1 << PORT1);
    PORTB &= ~(1 << PORT2);
    PORTB &= ~(1 << PORT3);
    PORTD &= ~(1 << PORT2);
    PORTD &= ~(1 << PORT3);
    delay_us_10us(pulse);
    PORTD &= ~(1 << PORT4);
    PORTD &= ~(1 << PORT5);
    PORTB &= ~(1 << PORT0);
    PORTB &= ~(1 << PORT1);
    delay_us_10us(10 - pulse);

    //Send reverse PWM output to Right Wheels until IMU reads correct angl
    i++;
  }
  //Send forward PWM output to All Wheels till on grid space
  //if(driveForward(speed)!=0){
  //  return 5;
  //}

  return 0;
}
