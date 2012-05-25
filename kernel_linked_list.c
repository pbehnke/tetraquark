/**************************************************************************
*	EGR 424 - Project 3
* 
* Phil Behnke
*
* A preemptive multithreaded kernel for the ARM Cortex M3 processor and EK-LM3S6965
* evaluation kit
*
* July 15, 2010
*
* Grand Valley State University
**************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "rit128x96x4.h"
#include "kernel.h"
#include "driverlib/systick.h"
#include "inc/lm3s6965.h"
#include "thread_list.h"
#include "main.h"

#define STACK_SIZE 2048   // Amount of stack space for each thread
/*
//Structure to hold information about each thread
typedef struct {
  int active;           // non-zero means thread is allowed to run
  char *stack;          // pointer to TOP of stack (highest memory location)
  unsigned state[40];   // space to save extra registers.
} threadStruct_t;

//Thread table containing the statically declared list of threads
//These are scheduled in a round-robin fashion
static thread_t threadTable[] = {
  thread1,
  thread2,
  thread3,
  thread4,
  thread5,
  idleThread
};
#define NUM_THREADS (sizeof(threadTable)/sizeof(threadTable[0]))
*/

// These static global variables are used in scheduler(), in
// the yield() function, and in threadStarter()
static threadStruct_t threads[NUM_THREADS]; // the thread table
unsigned currThread=-1;    // The currently active thread

//These functions need to be declared as naked to remove housekeeping functions
//performed by GCC.
void loadRegsFromStruc(unsigned i) __attribute__((naked)); //pure assembly function to load registers
void saveRegsOntoStruc(unsigned i) __attribute__((naked)); //pure assembly function to save registers
void scheduler(void) __attribute__((naked));               //scheduler function (systick ISR)
void svcInt(void) __attribute__((naked));                  //SVC interrupt handler


/**************************************************************************
*	This functions handles SVC interrupts.  It is called by the svcInt() function.
  The svcInt() function passes handleSVC() the code used in the SVC instruction.
**************************************************************************/
void handleSVC(int code)
{
  switch (code & 0xFF) {
    case 123:
      //set the pendstset bit to set a pending systick interrupt
      HWREG(0xE000ED04)|=(1<<26);
      break;

    default:
      iprintf("UNKNOWN SVC CALL\r\n");
      break;
  }
}

/**************************************************************************
*	This is the ISR for SVC instructions.  It gets the code used to generate the
  SVC instruction and passes it to handSVC() to do the actual work.
**************************************************************************/
void svcInt(void){
    asm volatile ("push {lr}\n");           //push the link register onto the stack
    asm volatile ("mrs r1, psp\n");         //load the psp into R1
    asm volatile ("LDR R0, [r1, #24]\n");   
    asm volatile ("SUB R0, R0, #2\n");      
    asm volatile ("LDRH R0, [R0]\n");       
    asm volatile ("AND R0, R0, #255\n");    //mask r0
    asm volatile ("bl handleSVC\n");        //branch to handleSVC with the code as the first parameter
    asm volatile ("pop {pc}\n");            //pop the LR into the PC to branch back to main in thread mode with the psp
}


/**************************************************************************
*	Initialize the system timer. 
 
	Parameters:
		ms - delay in between system tick interrupts in milliseconds
*
**************************************************************************/
void initSysTick(unsigned long ms){

	//gets the current clock frequency
	int clk_freq = SysCtlClockGet();

	//calculates the number of clock cycles needed to implement the new delay
	unsigned long numclocks = (((ms*1.0)/1000) * clk_freq);
	
	//sets the period of systick
	SysTickPeriodSet(numclocks);

	//Enable systick interrupts
	SysTickIntEnable();

	//Start systick
	SysTickEnable();

}


/**************************************************************************
*	This function changes to privilege by changing the control register.
**************************************************************************/
void changeControlReg(int cntrl){
	//moves the value passed in the parater cntrl into the control register.
	__asm__ __volatile__("MSR CONTROL,R0\n");
}

/**************************************************************************
*	This function saves R4-R11 and the process stack pointer into the array
  inside the structure for each thread.
**************************************************************************/
void saveRegsOntoStruc(unsigned i){

  __asm__ __volatile__("mrs r1, psp\n");              //store the psp into r1
  __asm__ __volatile__("stmea  r0!, {r1, r4-r11}\n"); //store r4-r11 and the psp into the array
  __asm__ __volatile__("bx lr\n");                    //branch back to scheduler()

}

/**************************************************************************
*	This function loads R4-R11 and the process stack pointer from the array
  inside the structure for each thread.
**************************************************************************/
void loadRegsFromStruc(unsigned i){

  __asm__ __volatile__("ldmia  r0!, {r1, r4-r11}\n"); //reload the values of r4-r11 and reload the psp into r1
  __asm__ __volatile__("msr psp, r1\n");              //move r1 into the psp
  __asm__ __volatile__("bx lr\n");                    //branch back to scheduler
 
}


/**************************************************************************
*	This function causes a systick ISR to be generated inside the handleSVC()
  function by causing an svc expection and passing handleSVC() a code of 123 
**************************************************************************/
void yield(void)
{
  __asm__ __volatile__("svc #123\n");   //trigger an svc exception with code 123
}


/**************************************************************************
*	This is the starting point for all threads. It runs in user thread
  context using the thread-specific stack. The address of this function
  is saved by createThread() in the LR field of the process stack so that
  the first time the scheduler() returns it will branch to the thread, we
  start here.
**************************************************************************/
void threadStarter(void)
{

  // Call the entry point for this thread. The next line returns
  // only when the thread exits.
  (*(threadTable[currThread]))();

  // Do thread-specific cleanup tasks. Currently, this just means marking
  // the thread as inactive. Do NOT free the stack here because we're
  // still using it! Remember, this function runs in user thread context.
  threads[currThread].active = 0;

  // This yield returns to the scheduler and never returns back since
  // the scheduler identifies the thread as inactive.
  yield();
}


/**************************************************************************
*	This function is implemented in assembly language in the file create.S.  
  This function sets up the initial stacks of each thread to branch to 
  threadStarter() when the systick expection exits.
**************************************************************************/
extern void createThread(unsigned *j, char *stack);

//Global counter variable used in the scheduler to keep track of how many
//threads are inactive
unsigned counter;

/**************************************************************************
*	This is the Systick ISR function.  It is the main scheduler of the kernel.
  It schedulers threads in a round robin fashion.
**************************************************************************/
void scheduler(void)
{

  //dont save registers the first time the first thread runs
  if(currThread!=(-1)){

    //save the values of R4-R11 and the PSP into the structure for the thread
    saveRegsOntoStruc(threads[currThread].state);
 
  }

  //store the number of threads inside counter
  counter=NUM_THREADS;

  //loop until there is an active thread or all the threads are inactive
  do{

    //increase the current thread
    if((++currThread) == NUM_THREADS)
    {
      currThread=0;
    }

	  //change to new thread by loading R4-R11 and changing the PSP to the new threads stack
    if(threads[currThread].active){
	    loadRegsFromStruc(threads[currThread].state);
      break;
    }

    //decrease the counter variable.
    counter--;

  }while(counter>0);
  
  
  if(counter==0){
    //all the threads are done, should never happen due to idle thread
  }

  //return to thread mode with the process stack pointer
  __asm__ __volatile__("ldr pc, =0xFFFFFFFD\n");

}

/**************************************************************************
*	This is the starting point of the kernel.
**************************************************************************/
void main(void)
{
  //looping variable
  unsigned i;

  // Set the clocking to run directly from the crystal.
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_8MHZ);

  // Initialize the OLED display and write status.
  RIT128x96x4Init(1000000);
  RIT128x96x4StringDraw("Preemptive Kernel!", 20,  0, 15);

  // Enable the peripherals.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  // Set GPIO A0 and A1 as UART pins.
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  // Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));

  threadList=createNode(thread1);

  // Create all the threads and allocate a stack for each one
  for (i=0; i < NUM_THREADS; i++) {
    // Mark thread as runnable
    threads[i].active = 1;

    // Allocate stack
    threads[i].stack = (char *)malloc(STACK_SIZE) + STACK_SIZE;

    //Check that malloc returned correctly
    if (threads[i].stack == STACK_SIZE) {
      RIT128x96x4StringDraw("ERROR!",20,50,15);
      iprintf("Out of memory\r\n");
      exit(1);
    }

    //After createThread() executes, each thread can be entered by loading 
    //R4-R11 and the psp from the threads structure and returning from an exception
    createThread(threads[i].state, threads[i].stack);
 
  }

  //start the systick timer with a 1 ms delay
  initSysTick(1);

  //change the control register to reduce the privilege and use the psp
  changeControlReg(0x03);

  //yield to set a systick exception
  yield();


}

/**************************************************************************
*	This function turns the on board LED on
**************************************************************************/
void LEDon(void){
  HWREG(0x40025004)=0x1;
}

/**************************************************************************
*	This function turns the on board LED off
**************************************************************************/
void LEDoff(void){
  HWREG(0x40025004)=0x0;
}

/**************************************************************************
*	This function initializes the on board LED
**************************************************************************/
void LEDinit(void){

  volatile unsigned long ulLoop;
    
  //enable the GPIO port
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;

  //delay for a few cycles
  ulLoop = SYSCTL_RCGC2_R;

  // Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
  // enable the GPIO pin for digital function.
  GPIO_PORTF_DIR_R = 0x01;
  GPIO_PORTF_DEN_R = 0x01;

}


/**************************************************************************
*	Sets PORTE PIN3 as an output and sets the pin high.  Useful to check the timing
  of the kernel using an oscilliscope
**************************************************************************/
void initPins(void){

  //enable the GPIO port
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;

	//Setup pins as outputs
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3); 

  //set the pin high
  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0xFF);
}

/*
 * Compile with:
 * ${CC} -o crsched.elf -I${STELLARISWARE} -L${STELLARISWARE}/driverlib/gcc 
 *     -Tlinkscript.x -Wl,-Map,crsched.map -Wl,--entry,ResetISR 
 *     crsched.c create.S threads.c startup_gcc.c syscalls.c rit128x96x4.c 
 *     -ldriver
 */
// vim: expandtab ts=2 sw=2 cindent
