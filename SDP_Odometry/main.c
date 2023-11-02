/*
Odometry 2022-2023
John Boesen
Main program for motor driving/odometry

Routine:
Accept commands through the Serial Bus from NUC
    -grab data from subscriber ROS node on NUC
Interpret and preform commands
    -List of commands in movement.c
(maybe)Send acknoledgement signal to NUC through Serial
    -200 Ok, 100 In Progress, 001 Program Halted

TODO
- Test USART Communication
- Test ROS Listener Node
- Implement PS4 Controller Code from github
- (Maybe)Create signal handler to queue incoming commands

USART code based off of https://github.com/xanthium-enterprises/atmega328p-serial-uart-to-pc-communication/blob/master/_2_Serial_Receive/main.c
https://www.xanthium.in/how-to-avr-atmega328p-microcontroller-usart-uart-embedded-programming-avrgcc
*/

//#include "readIMU.cpp"
#include "movement.h"
#include <string.h>

#define F_CPU 16000000
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE 103

char USART_readchar(){
  while (!(UCSR0A & (1<<RXC0)));
  return UDR0;
}

char USART_test(int i,int j){
  char data[] = {'0',' ','0','.','5','\n'};
  if (i == 0){
      return data[j];
  }else{
      return data[j+i];
  }        
}

///*
int main (void){
  unsigned char data[3];
  float cmd [3];
  const char s = ' ';

 // UCSR0B |= (1 << RXEN0) | (1 << TXEN0);   // Turn on the transmission and reception circuitry
  UCSR0B = 0b00011000;
  UCSR0C = 0x06; 

  //Configure UART
  UBRR0L = BAUD_PRESCALE;
  UBRR0H = (BAUD_PRESCALE >> 8);

  while(1){ // Format for data recieve is 'cmd angle speed'
    ///*
    int i = 0;
    int j = 0;
    /* // Test code for switch/case and float conversion
    data[0] = "0";
    data[1] = ".";   
    data[2] = "5";
    /**/
    while (i<3){ // recieve/interpret USART data
        data[j] = USART_readchar();
        //data[j] = USART_test(i,j);
      if (data[j] == ' '){
        cmd[i] = atof(data);
        j=0;
        i++;
      } else if (data[j] == '\n'){
        cmd[i] = atof(data);
        break;            
      } else {
        j++;
      }
    }
    //*/
    /*  //Test code for switch/case
    cmd[0] = atof("0");
    cmd[1] = atof(data);
    cmd[2] = atof("1.57");
    */
    switch((int)cmd[0]){ //Preform action
      case 0:
        driveForward(cmd[2]);
        break;
      case 1:
        driveRight(cmd[1],cmd[2]);
        break;
      case 2:
        driveLeft(cmd[1],cmd[2]);
        break;
      case 3:
        //digCycle();
        break;
      //default:
        //driveForward(0.5);
        //driveRight(0.5,3.14);
        //driveForward(0.5);
        //break;          
    }
  }
}
//*/

//Test Code for movement.c
/*
int main(void){
    while (1){
      driveForward(0.5);
      _delay_us(10000);
      driveRight(3.14/2,0.5);
      _delay_us(1000);
      driveLeft(3.14/2,0.5);
      _delay_us(10000);
    }   
}*/