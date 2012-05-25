/**************************************************************************
* Threads.c
*
* EGR 424- Project 3
*
* Phil Behnke
**************************************************************************/
#include <stdio.h>
#include "kernel.h"
#include "rit128x96x4.h"
#include "inc/lm3s6965.h"
#include "inc/hw_types.h"
#include "main.h"

/**************************************************************************
* These are the user-space threads. Note that they are completely oblivious
  to the technical concerns of the scheduler. 
**************************************************************************/


/**************************************************************************
* This is the first thread.  It blinks the LED using the LED API from the kernel.
**************************************************************************/
void thread1(void)
{
    volatile unsigned long ulLoop;

    //initialize the LED
    LEDinit();

    //loop forever
    while(1)
    {
        // Turn on the LED.
	LEDon();
	
        // Delay for a bit.
        for(ulLoop = 0; ulLoop < 200000; ulLoop++)
        {
        }

        // Turn off the LED.
	LEDoff();
	
        // Delay for a bit.
        for(ulLoop = 0; ulLoop < 200000; ulLoop++)
        {
        }
    }
}

/**************************************************************************
* This is the second thread.  It prints to the UART.
**************************************************************************/
void thread2(void)
{
  //loop forever
  while(1){

    //write to the UART
    iprintf("\nIn thread 2\r\n");

    //yield to the kernel
    yield();

  }
}


/**************************************************************************
* This is the third thread.  It prints to the UART.
**************************************************************************/
void thread3(void)
{
  //loop forever
  while(1){

    //write to the UART
    iprintf("\nIn thread 3\r\n");

    //yield to the kernel
    yield();

  }
}

/**************************************************************************
* This is the fourth thread. It writes to the UART.
**************************************************************************/
void thread4(void)
{

  //loop forever
  while(1){

    //write to the UART
    iprintf("\nIn thread 4\r\n");

    //yield to the kernel
    yield();

  }  
}

/**************************************************************************
* This is the fifth thread.  It writes to the OLED display on the EK-LM3S6965 using 
* the StellarisWare drivers.
**************************************************************************/
void thread5(void){
  unsigned count;
  unsigned j;
  unsigned i;

  //flash the display 50 times
  for(i=0; i<50; i++){
    //draw to and clear the display
    for (count=0; count < 2; count++) {

      if(count){
	RIT128x96x4StringDraw("TetraQuark Test",20,60,15);
      }
      else{
	RIT128x96x4Clear();
      }

      //delay for a bit
      for(j=0;j<50000;j++);
    }
  }
  
}

/**************************************************************************
* This is the idle thread.  It runs when there is no other work to be done.
**************************************************************************/
void idleThread(void)
{
	while(1);
}

