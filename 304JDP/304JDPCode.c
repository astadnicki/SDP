#include <avr/io.h>
#include <util/delay.h>
#include "i2c.h"
#include "SSD1306.h"

#define DELAY 5
//echo D4, trig D5 for sensor 0
//echo B4, trig B3 for sensor 1
#define TRIG0 5 //flipped sensors cuz they were wrong
#define ECHO0 4
#define TRIG1 4
#define ECHO1 3
#define RANGE_PER_CLOCK 1.098

int PEOPLE = 0;
int MAX = 100;

int TMP_READ(void){
  int saveD;
  int saveB;
  int savePB;

  //PORTD is bottom and PORTB is top
  saveD = DDRD;
  saveB = DDRB;
  savePB = PORTB;

  DDRD = 0xFF;
  DDRB = 0xFE;
  PORTB = 0xFE;
  unsigned char ledDigits[] = {0x7B,0x48,0x67,0x6E,0x5C,0x3E,
    0x3F,0x68,0x7F,0x7C};
  double num;
  unsigned char DIG1,DIG2,DIG3;

  unsigned int digitalValue;

  unsigned int i = 0;

  i++;
  if(i>9999) i = 0;

  DDRC = 0x00; //set PD0 = ADC0 as input
  ADMUX = 0xC0; //select ADC0; Vref=5V
  ADCSRA = 0x87; //enable ADC; speed 125Khz
  ADCSRA |= (1<<ADSC); //start ADC conversion
  //delay with counter
  if (i % 20 == 0){
      while ((ADCSRA & (1<<ADIF)) == 0); //wait until finished converting
      //If button is pushed, switch to celsius
      if ((PINB & (1<<PINB5)) == 0){ //PINC1 is 0 when pressed
          digitalValue = ((ADCL|(ADCH<<8))/10 - 32)*50/9; //convert ADC digital value to celsius
      } else {
          digitalValue = ADCL|(ADCH<<8); //read the ADC digital value
      }
  }

  for (int j=0; j<10; j++){ // Sample tempreture every 125 ms (Uses the 4 digit ring counter from lecutre as base)

    DIG1 = ((int) num/100) % 10; //Display tempreture readout
    PORTD = ledDigits[DIG1];
    PORTB =~ (1<<1);
    _delay_ms(DELAY);

    DIG2 = ((int) num/10) % 10;
    PORTD = ledDigits[DIG2];
    PORTD |= 0x80;
    PORTB =~ (1<<2);
    _delay_ms(DELAY);

    DIG3 = (int) num % 10;
    PORTD = ledDigits[DIG3];
    PORTB =~ (1<<3);
    _delay_ms(DELAY);
  }
  DDRD = saveD;
  DDRB = saveB;
  PORTB = savePB;
  return num;
}

void displayPeople(void){
  OLED_DisplayString("Number of People: ");
  OLED_DisplayNumber(10,PEOPLE,2);
  _delay_ms(100);
  /*
  if (PEOPLE >= MAX){      //sets led to red 
    PORTD |= (1<<PORTD2);
    PORTD &= ~(1<<PORTD3);
    PORTD &= ~(1<<PORTD7);
    _delay_ms(100);
  }
  else if (((MAX/2)>PEOPLE)){ //sets led to green
    PORTD |= (1<<PORTD3);
    PORTD &= ~(1<<PORTD2);
    PORTD &= ~(1<<PORTD7);
    _delay_ms(100);
  }
  else if ((MAX>PEOPLE)){   //sets led to yellow
    PORTD |= (1<<PORTD7);
    PORTD &= ~(1<<PORTD3);
    PORTD &= ~(1<<PORTD2);
    _delay_ms(100);
  }
  else{
    PORTD &= ~(1<<PORTD2);  //figure out led ports
    PORTD &= ~(1<<PORTD3);
    PORTD &= ~(1<<PORTD7);
    _delay_ms(100);
  }
  */
}
void displaySTR(char* str){
  OLED_DisplayString(str);
  _delay_ms(100);
}


float findDistance(int type){ //type is a 1 or 0, each has a different sensor
  //echo D4, trig D5 for sensor 0
  //echo B4, trig B3 for sensor 1

  int rising_edge, falling_edge, echo_width, target_range;
  if (type == 0){
    PORTD &= ~(1 << TRIG0); // 5 usec pre-TRIG
    _delay_us(5);
    PORTD |= (1 << TRIG0); // 10 us pulse
    _delay_us(10); // to ultrasound
    PORTD &= ~(1 << TRIG0); // TRIG pin
    TCNT0 = 0;
    // Wait till ECHO pulse goes high
    while((PIND & (1<<ECHO0)) == 0);
    OLED_GoToLine(1);
    OLED_DisplayString("you");
    rising_edge = TCNT0; // Note the time
    // Now wait till ECHO pulse low
    while (!(PIND & (1<<ECHO0)) == 0);
    falling_edge = TCNT0;
    if(falling_edge > rising_edge){
      //Compute target range and send to serial monitor
      echo_width = falling_edge - rising_edge;
      target_range = echo_width * RANGE_PER_CLOCK;
      // calculates as float first to be more accurate in the inches
      // displays at int to be more legible
      float cm = (target_range*1.098);    // distance in centimeters
      return cm;
    }
  }
  else if(type == 1){
    PORTB &= ~(1 << ECHO1); // 5 usec pre-TRIG
    _delay_us(5);
    PORTB |= 1 << ECHO1; // 10 us pulse
    _delay_us(10); // to ultrasound
    PORTB &= ~(1 << ECHO1); // TRIG pin
    TCNT0 = 0;
    // Wait till ECHO pulse goes high
    while((PINB & (1<<TRIG1)) == 0);
      rising_edge = TCNT0; // Note the time
    // NOw wait till ECHO pulse low
    while (!(PINB & (1<<TRIG1)) == 0);
      falling_edge = TCNT0;
    if(falling_edge > rising_edge){

      //Compute target range and send to serial monitor
      echo_width = falling_edge - rising_edge;
      target_range = echo_width * RANGE_PER_CLOCK;
      // calculates as float first to be more accurate in the inches
      // displays at int to be more legible
      float cm = (target_range*1.098);    // distance in centimeters
      return cm;
    }
  }
  return 0;
}

void lockDoor(void){
  for (int i=0;i<10;i++){
    PORTC |= (1<<PORTC3);
    _delay_ms(0.5);
    PORTC &= ~(1<<PORTC3);
    _delay_ms(4.5);
  }
  PORTC &= ~(1<<PORTC3);
  displaySTR("Door Locked");
}

void unlockDoor(void){
  for (int i=0;i<10;i++){
    PORTC |= (1<<PORTC3);
    _delay_ms(3);
    PORTC &= ~(1<<PORTC3);
    _delay_ms(2);
  }
  PORTC &= ~(1<<PORTC3);
  displaySTR("Door Unlocked");
  _delay_ms(200);
  lockDoor();
}


void timer_init(){
    TCCR0A = 0XA3;     // Timer 1 Normal mode (count up)
    TCCR0B = 5;     // Divide clock by 1024
    TCNT0 = 0;      // Start the timer at 0
}

int main(void) {
  float dis;
  int temp;

  DDRB = 0xFF; 
  //DDRC = 1<<PORTC5 | 1<<PORTC3 | 1<<PORTC0;
  //Analog Ports
  //PORTC = 1<<PORTC4 | 1<<PORTC2 | 1<<PORTC1;
  DDRD = 0xFF;

  timer_init();
  OLED_Init();
  while(1){
    OLED_GoToLine(0);
    OLED_DisplayString("fuck");
    _delay_ms(10);
    dis = findDistance(0);
    OLED_GoToLine(1);
    OLED_DisplayNumber(10, dis, 3);
    if (dis<10){
      OLED_GoToLine(0);
      displaySTR("Please Touch Temp Sensor");
      temp = TMP_READ();
      OLED_GoToLine(1);
      OLED_DisplayNumber(10, temp, 2);
      _delay_ms(200);
      if (temp>99){
        OLED_GoToLine(0);
        displaySTR("Enter only if you can verify that you aren't sick");
        _delay_ms(1000);
      } else {
        OLED_GoToLine(0);
        displaySTR("Please Enter");
      }
      unlockDoor();
      lockDoor();
    }
    dis = findDistance(1);
    if (dis<10){
      unlockDoor();
      lockDoor();
    }
    displayPeople();
    
  }
}

