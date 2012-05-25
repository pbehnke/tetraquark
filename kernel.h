/**************************************************************************
* Header file for the kernel.  These are the public functions that the threads
* can access.
**************************************************************************/

#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

extern unsigned currThread;	//Number of the current thread (starting at 0)
extern void yield(void);	//Yield to the kernel
extern void LEDon(void);	//Turn the LED on
extern void LEDoff(void);	//Turn the LED off
extern void LEDinit(void);	//Initialize the LED

#endif // _SCHEDULER_H_
