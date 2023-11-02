#include <avr/io.h>
#include <util/delay.h>
#define MYDELAY 100

int main(void) {
  
  DDRB = 1<<PB5;

  while(1){
    PORTB = 1<<PB5;
    _delay_ms(MYDELAY);
    PORTB = ~ (1<<PB5);
    _delay_ms(MYDELAY);
  }

  return 0;

}

// cd Downloads/SDP_Odometry/blinktest
// avr-gcc -Wall -Os -DF_CPU=16000000 -mmcu=atmega328p -o blink.elf blink.c
// avr-objcopy -O ihex blink.elf blink.hex
// avrdude -c arduino -b 115200 -P COM4 -p atmega328p -U flash:w:blink.hex:i