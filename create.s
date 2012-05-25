/*
 * Implement the thread creation task:
 *
 *   - initialize the stack with appropriate values for
 *     R13 (stack) and R14 (first address to jump to) and xPSR (0x1000000)
 *   - all other registers are irrelevant upon thread creation
 *
 *   The stack is set to the second parameter of this
 *   function (the top-of-stack address, passed in R1). The R14 slot is set to
 *   the address of the threadStarter() function.
 *
 *   The C prototype for this function call is:
 *       createThread(threads[i].state, threads[i].stack)
 *   thus:
 *       R0 <-- state (a setjmp()-style jump buffer)
 *       R1 <-- stack (address of top-of-stack)
 */
    .syntax unified
    .text
    .align 2
    .thumb
    .thumb_func
    .type createThread,function
    .global createThread
createThread:  

  /* Save registers in on the stack. */
    
    mov r3, #0	@moves 0 into r3
    sub r1, r1, #32	@subtracts 32 bytes from the stack pointer
    stmea  r0!, { r1 ,r4-r11 }	@stores the new stack pointer and the values of R4-R11 into the state array
    stmia  r1!, { r3 }	@stores a 0 in R0's position on the stack
    stmia  r1!, { r3 }	@stores a 0 in R1's position on the stack
    stmia  r1!, { r3 }	@stores a 0 in R2's position on the stack
    stmia  r1!, { r3 }	@stores a 0 in R3's position on the stack
    stmia  r1!, { r3 }	@stores a 0 in R12's position on the stack
    ldr    r3, .L1		@stores the address of yield in r3
    stmia  r1!, { r3 }	@stores the address of yield in the LR position on the stack
    ldr     R3, .L0		@sotres the address of threadStarter in r3
    stmia  r1!, { r3 }	@stores the address of threadStarter into the address to return to position on the stack
    mov r3, #0x1000000	@stores 0x1000000 into r3
    stmia  r1!, { r3 }	@stores 0x1000000 into the xPSR position on the stack.  This sets the 'T' bit in the xPSR
    bx      lr			@branch back to main
    
.L0:
    .word   threadStarter
.L1:
    .word   yield
