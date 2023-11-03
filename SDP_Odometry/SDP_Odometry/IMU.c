/*
#include "MPU6050.h"
#include <avr/io.h>
#include <util/delay.h>
#define MYDELAY 8
*/

int main (void){

  unsigned int digitalValue;

  // IMU Code
  // SCL in A5 is the I2C serial clock, SDA is the I2C serial data in A4 (testing in A0)
  
  DDRC = 0x00; //set PD0 = A0
  ADMUX = 0xC0; //select ADC0; Vref=5V
  ADCSRA = 0x87; //enable ADC; speed 125Khz
  ADCSRA |= (1<<ADSC); //start ADC conversion
  //delay with counter
  if (i % 20 == 0){
      while ((ADCSRA & (1<<ADIF)) == 0); //wait until finished converting
      digitalValue = ADCL|(ADCH<<8); //read the ADC digital value
  }

  printf("%d", digitalValue)  // testing value of A4

  while (1){

    // Computing roll and pitch from accelerometer
    

  }



}





// cd Downloads/SDP_Odometry
// avr-gcc -Wall -Os -DF_CPU=16000000 -mmcu=atmega328p -o IMU.elf IMU.c
// avr-objcopy -O ihex IMU.elf IMU.hex
// avrdude -c arduino -b 115200 -P COM4 -p atmega328p -U flash:w:IMU.hex:i

// avr-gcc -Wall -Os -DF_CPU=16000000 -mmcu=atmega328p -o main.elf main.c
// avr-objcopy -O ihex main.elf main.hex
// avrdude -c arduino -b 115200 -P COM4 -p atmega328p -U flash:w:main.hex:i