
#include "movement.h"
#define RESOLUTION 5


int driveForward(float speed){
    //Set Output/Input pins
    DDRD = 1<<PIN2|1<<PIN3|1<<PIN4|1<<PIN5;
    //Send forward PWM output to All Wheels till on grid space 
    float pulse = speed*999 + 1000;

    //PIN8, PIN9, PIN10, PIN11
    //PORTB: 0

    int i=0;
    while (i< 10000){
        PORTD |= (1<<PORT2 || 1<<PORT3 || 1<<PORT4 || 1<<PORT5);
        _delay_us(pulse);
        PORTD &= ~(1<<PORT2 || 1<<PORT3 || 1<<PORT4 || 1<<PORT5);
        _delay_us(2000-pulse);
        i++;
    }

    return 0;
}

int driveReverse(float speed){
    //Set Output/Input pins
    DDRB = 1<<PIN0|1<<PIN1|1<<PIN2|1<<PIN3;
    //Send forward PWM output to All Wheels till on grid space 
    float pulse = speed*999 + 1000;
    //PIN8, PIN9, PIN10, PIN11
    //PORTB: 0
    int i=0;
    while (i< 10000){
        PORTB |= (1<<PORT0 || 1<<PORT1 || 1<<PORT2 || 1<<PORT3);
        _delay_us(pulse);
        PORTB &= ~(1<<PORT0 || 1<<PORT1 || 1<<PORT2 || 1<<PORT3);
        _delay_us(2000-pulse);
        i++;
    }

    return 0;
}


int driveRight(float deg, float speed){
    //Set Output/Input pins
    DDRD = 1<<PIN2|1<<PIN3;
    float pulse = speed*999 + 1000;
    int i=0;
    while (i<10000){
        PORTD |= (1<<PORT2);
        PORTD |= (1<<PORT3);
        _delay_us(pulse);
        PORTD &= ~(1<<PORT2 || 1<<PORT3);
        _delay_us(2000-pulse);
        i++;
    }

    return 0;
}

int driveLeft(float deg, float speed){
    //Set Output/Input pins
    DDRD = 1<<PIN4|1<<PIN5;
    float pulse = speed*999 + 1000;
    int i = 0;
    while (i<10000){
        PORTD |= (1<<PORT4);
        PORTD |= (1<<PORT5);
        _delay_us(pulse);
        PORTD &= ~(1<<PORT4 || 1<<PORT5);
        _delay_us(2000-pulse);
        i++;
   }

    return 0;
}