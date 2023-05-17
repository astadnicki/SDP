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
  //PORTD is bottom and PORTB is to

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
      digitalValue = ADCL|(ADCH<<8); //read the ADC digital value
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
  return num;
}

void displayPeople(void){
  //OLED_Clear();
  OLED_GoToLine(2);
  OLED_DisplayString("Number of People: ");
  OLED_GoToLine(3);
  OLED_DisplayNumber(10,PEOPLE,2);
  _delay_ms(100);//*/
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

void clearline(int line){
  OLED_GoToLine(line);
  OLED_DisplayString("                    ");
}


float findDistance(int type){ //type is a 1 or 0, each has a different sensor
  //echo D4, trig D5 for sensor 0
  //echo B4, trig B3 for sensor 1
  DDRB = 1<< PIN4; // Trig0
  DDRD = 1<< PIN5; // Trig1
  PORTB = 1<< PORTB3; //ECHO0
  PORTD = 1<< PORTD4 ;//ECHO1 

  int rising_edge, falling_edge, echo_width, target_range;
  if (type == 0){
    PORTB &= ~(1 << PIN4); // 5 usec pre-TRIG
    _delay_us(5);
    PORTB |= (1 << PIN4); // 10 us pulse
    _delay_us(10); // to ultrasound
    PORTB &= ~(1 << PIN4); // TRIG pin
    TCNT0 = 0;
    // Wait till ECHO pulse goes high
    while((PINB & (1<<PORTB3)) ==0);
      
      rising_edge = TCNT0; // Note the time
    // Now wait till ECHO pulse low
    while (!(PINB & (1<<PORTB3)) ==0);
      falling_edge = TCNT0;
    if(falling_edge > rising_edge){
      //Compute target range and send to serial monitor
      echo_width = falling_edge - rising_edge;
      target_range = echo_width * RANGE_PER_CLOCK;
      // calculates as float first to be more accurate in the inches
      // displays at int to be more legible
      float cm = (target_range*1.098);    // distance in centimeters
      //OLED_GoToLine(0);
      //displaySTR("Fuck you0");
      return cm;
    }
  }
  else if(type == 1){
    PORTD &= ~(1 << PIN5); // 5 usec pre-TRIG
    _delay_us(5);
    PORTD |= (1 << PIN5); // 10 us pulse
    _delay_us(10); // to ultrasound
    PORTD &= ~(1 << PIN5); // TRIG pin
    TCNT0 = 0;
    // Wait till ECHO pulse goes high
    while((PIND & (1<<PORTD4)) ==0);
      
      rising_edge = TCNT0; // Note the time
    // Now wait till ECHO pulse low
    while (!(PIND & (1<<PORTD4)) ==0);
    //OLED_GoToLine(0);
    //displaySTR("Fuck you1");
    
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
  clearline(0);
  OLED_GoToLine(0);
  displaySTR("Door Locked");
  _delay_ms(10000);
  clearline(0);
}

void unlockDoor(void){
  for (int i=0;i<10;i++){
    PORTC |= (1<<PORTC3);
    _delay_ms(3);
    PORTC &= ~(1<<PORTC3);
    _delay_ms(2);
  }
  PORTC &= ~(1<<PORTC3);
  clearline(0);
  OLED_GoToLine(0);
  displaySTR("Door Unlocked");
  _delay_ms(10000);
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
  timer_init();
  OLED_Init();
  //OLED_GoToLine(0);
  //OLED_DisplayString("fuck");

  DDRB = 0xFF; 
  //DDRC = 1<<PORTC5 | 1<<PORTC3 | 1<<PORTC0;
  //Analog Ports
  //PORTC = 1<<PORTC4 | 1<<PORTC2 | 1<<PORTC1;
  DDRD = 0xFF;

  
  
  while(1){
    //clearline(0);
    
    _delay_ms(10);
    dis = findDistance(0);
    //OLED_GoToLine(1);
    //OLED_DisplayNumber(10, dis, 3);
    if (dis<10){
      clearline(0);
      OLED_GoToLine(0);
      displaySTR("Enter only if you can verify that you aren't sick (Press Button)");
      //displaySTR("Touch Temp Sensor");
      while ((PINB & (1<<PIN5)) != 0);
      //temp = TMP_READ();
      //OLED_GoToLine(1);
      //OLED_DisplayNumber(10, temp, 3);
      //_delay_ms(200);
      //if (temp>70){
      //_delay_ms(10000);
      //if (PINB & (1<<PIN5) != 1){
      displaySTR("Thank you! Please enter");
      PEOPLE ++;
      //}
      _delay_ms(10000);
      /*} else {
        clearline(0);
        OLED_GoToLine(0);
        displaySTR("Please Enter");
        PEOPLE ++;
        _delay_ms(10000);
      }*/
      unlockDoor();
      lockDoor();
    }
    dis = findDistance(1);
    
    if (dis<10){
      unlockDoor();
      PEOPLE --;
      lockDoor();
    }
    displayPeople();
    
  }
}