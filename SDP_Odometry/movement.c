/*
Odometry 2022-2023 WIP
John Boesen
Collection of Functions to Drive Robot Motors/Acutators

PIND 2 - Forward Right
PIND 3 - Rear Right
PIND 4 - Forward Left
PIND 5 - Rear Left
PIND 6 - Dig Motors
PIND 7 - Linear Actuators

PINB 0 - IMU SDA
PINB 1 - IMU SCL
PINB 2 - Right Linear Actuator Pot Ref
PINB 3 - Right Linear Actuator Pot Signal
PINB 4 - Left Linear Actuator Pot Ref
PINB 5 - Left Linear Actuator Pot Signal

Test Order:
-readIMU() in readIMU.cpp
-xforward()
-xdriveRight()/driveLeft()
-xdigCycle()
-extend()/contract()

TODO
-Figure out PWM timing for actuators
-Determine how to read/interpret IMU data through I2C (Use C++ Library)
-Testing (Forward,Left,Right work w/o IMU)

*/
//#include "movement.h"
#define RESOLUTION 5
#define __DELAY_BACKWARD_COMPATIBLE__

void delay_us_10us(int ms) // variable compatible delay wrapper
{
  while (0 < ms)
  {  
    _delay_us(10);
    ms -= 10;
  }
}



int driveForward(float speed){
    //Set Output/Input pins
    DDRD = 1<<PIN2|1<<PIN3|1<<PIN4|1<<PIN5;
    //Send forward PWM output to All Wheels till on grid space 
    int pulse;
    float v0=0;
    //float a = readIMU(1);
    //clock_t prevTime;
    //clock_t stTime;

    if (speed < 0){
        pulse = (1+speed)*474+1000;
    } else{
        pulse = (speed)*474+1525;
    }
    //float dist = 0;
    int i=0;
   /* while(dist>RESOLUTION){
        prevTime=clock();
        PORTD |= (1<<PORT2 || 1<<PORT3 || 1<<PORT4 || 1<<PORT5);
        _delay_us(pulse);
        PORTD &= ~(1<<PORT2 || 1<<PORT3 || 1<<PORT4 || 1<<PORT5);
        _delay_us(2000-pulse);
        stTime=clock()-prevTime;
        dist = v0*stTime+0.5*a*stTime*stTime;
        v0 = v0+a*stTime;
        a=readIMU(1);
    }*/
    while (i< 1000){ //Drive w/o IMU data (Only use for tele-op)
        PORTD |= (1<<PORT2);
        PORTD |= (1<<PORT3);
        PORTD |= (1<<PORT4);
        PORTD |= (1<<PORT5);
        delay_us_10us(pulse);
        PORTD &= ~(1<<PORT2);
        PORTD &= ~(1<<PORT3);
        PORTD &= ~(1<<PORT4);
        PORTD &= ~(1<<PORT5);
        delay_us_10us(2000-pulse);  
        i++;
    }

    return 0;
}


int driveRight(float ang, float speed){
    
    if (ang>3.14 || ang<0){
      return 1;
    }
    if(speed>1){
      return 2;
    } else if(speed<0){
      return 3;
    }

    float angle;
    
    //Set Output/Input pins
    DDRD = 1<<PIN2|1<<PIN3|1<<PIN4|1<<PIN5;

    //Setup pulse and angle variables
    int rpulse = (1-speed)*474 + 1000;
    int fpulse = speed*474 + 1525;
    //angle = readIMU(0);

    //while(angle<ang){

        //Send forward PWM output to Left Wheels until IMU reads correct angle
        PORTD |= (1<<PORT2);
        PORTD |= (1<<PORT3);
        delay_us_10us(rpulse);
        PORTD &= ~(1<<PORT2);
        PORTD &= ~(1<<PORT3);
        delay_us_10us(2000-rpulse);

        //Send reverse PWM output to Right Wheels until IMU reads correct angle
        PORTD |= (1<<PORT4);
        PORTD |= (1<<PORT5);
        delay_us_10us(fpulse);
        PORTD &= ~(1<<PORT4);
        PORTD &= ~(1<<PORT5);
        delay_us_10us(2000-fpulse);

        //Grab IMU data to check angle
        //angle = readIMU(0);
        delay_us_10us(fpulse+rpulse);
    //}
    
    //Send forward PWM output to All Wheels till on grid space 
    //if(driveForward(speed)!=0){
        //return 5;
    //}

    return 0;
}

int driveLeft(float deg, float speed){
    if (deg>3.14 || deg<0){
        return 1;
    }
    if(speed>1){
        return 2;
    } else if(speed<0){
        return 3;
    }

    float angles[3];
    
    //Set Output/Input pins
    DDRD = 1<<PIN2|1<<PIN3|1<<PIN4|1<<PIN5;

    //Setup pulse and angle variables
    int rpulse = (1-speed)*474 + 1000;
    int fpulse = speed*474 + 1525;
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
   int i = 0; //Drive w/o IMU data (Only use for tele-op)
   while (i<1000){
        PORTD |= (1<<PORT2);
        PORTD |= (1<<PORT3);
        delay_us_10us(fpulse);
        PORTD &= ~(1<<PORT2);
        PORTD &= ~(1<<PORT3);
        delay_us_10us(2000-fpulse);

        //Send reverse PWM output to Right Wheels until IMU reads correct angle
        PORTD |= (1<<PORT4);
        PORTD |= (1<<PORT5);
        delay_us_10us(rpulse);
        PORTD &= ~(1<<PORT4);
        PORTD &= ~(1<<PORT5);
        delay_us_10us(2000-rpulse);
        i++;

   }
    //Send forward PWM output to All Wheels till on grid space 
    //if(driveForward(speed)!=0){
      //  return 5;
    //}

    return 0;
}


int extend(void){
    float perExtend= 0.5;
    //Set Output/Input pins
    DDRD |= (1<<PORT7);
    PORTB |= (1<<PORT2 || 1<<PORT3 || 1<<PORT4 || 1<<PORT5);
    //Send extend signal to actuators

    ADMUX = 0xC0;
    ADCSRA = 0x87;
    ADCSRA |= (1<<ADSC);
    while ((ADCSRA & (1<<ADIF)) == 0);
    float pot = (ADCL | (ADCH << 8))/1023;

    while (pot<perExtend){
        PORTD |= (1<<PORT7);
        _delay_us(1000); //Figure out timing for extend
        PORTD &= ~(1<<PORT7);
        _delay_us(3000);
        pot = (ADCL | (ADCH << 8))/1023;
    }
    
    return 0;
}

int contract(void){
    //Set Output/Input pins
    //Send contract signal to actuators

  float perExtend = 0.5;

    PORTB |= (1<<PORT2 || 1<<PORT3 || 1<<PORT4 || 1<<PORT5);
    //Send extend signal to actuators

    ADMUX = 0xC0;
    ADCSRA = 0x87;
    ADCSRA |= (1<<ADSC);
    while ((ADCSRA & (1<<ADIF)) == 0);
    float pot = (ADCL | (ADCH << 8))/1023;

    while (pot<perExtend){
        PORTD |= (1<<PORT7);
        _delay_us(1000); //Figure Out timing for contract
        PORTD &= ~(1<<PORT7);
        _delay_us(3000);
        pot = (ADCL | (ADCH << 8))/1023;
    }

    return 0;
}

int digCycle(void){
    //Set Output/Input Pins
    //Send Forward PWM output to 2 dig motors
    //Wait x seconds
    DDRD = 1<<PIN6;

    //extend();

    for (int i=0;i<10000;i++){ //Runs for 25 sec
        PORTD |= (1<<PORT6);
        _delay_us(2000);
        PORTD &= ~(1<<PORT6);
        _delay_us(500);
    }

    //contract();

    return 0;
}

