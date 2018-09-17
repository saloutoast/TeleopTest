//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hello World (hello)</h1>
//!
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the UART and is a starting point for more complicated applications.
//!
//! UART0, connected to the Virtual Serial Port and running at
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

static volatile int controller_flag = 0;

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

// Set up encoder modules
void ConfigureQEI0(void) {
  // Enable peripherals
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

  // from "https://forum.43oh.com/topic/7170-using-harware-qei-on-tiva-launchpad/":

  //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

  // Set pins for QEI0
  GPIOPinConfigure(GPIO_PD6_PHA0);
  GPIOPinConfigure(GPIO_PD7_PHB0);
  //GPIOPinConfigure( index );

  //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
	GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);

	//DISable peripheral and int before configuration
	QEIDisable(QEI0_BASE);
	QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

	// Configure quadrature encoder, use an arbitrary top limit of 1000
	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 8191);
  QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, 500000);

	// Enable the quadrature encoder.
	QEIEnable(QEI0_BASE);
  QEIVelocityEnable(QEI0_BASE);

	//Set position to a middle value so we can see if things are working
	QEIPositionSet(QEI0_BASE, 4096);
}

void ConfigureQEI1(void) {
  // Enable peripherals
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

  // Set pins for QEI1
  GPIOPinConfigure(GPIO_PC5_PHA1);
  GPIOPinConfigure(GPIO_PC6_PHB1);
  //GPIOPinConfigure( index );


  // from "https://forum.43oh.com/topic/7170-using-harware-qei-on-tiva-launchpad/":

  //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);

	//DISable peripheral and int before configuration
	QEIDisable(QEI1_BASE);
	QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

	// Configure quadrature encoder, use an arbitrary top limit of 1000
	QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 8191);
  QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, 500000);

	// Enable the quadrature encoder.
	QEIEnable(QEI1_BASE);
  QEIVelocityEnable(QEI1_BASE);

	//Set position to a middle value so we can see if things are working
	QEIPositionSet(QEI1_BASE, 4096);
}



// Timer init code from "https://gist.github.com/robertinant/10398194" and "timers.c" in examples folder
void Timer0ISR(void)
{
  ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  // Clear the timer interrupt

  // set global flag for new control values
  if(controller_flag==0) {
    controller_flag = 1;
  }

  // toggle an LED here?

}

void initTimer(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // enable peripheral
  ROM_IntMasterEnable(); // enable processor interrupts
  ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);   // 32 bits Timer
  ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()/2); // set to roll over at 10Hz
  ROM_IntEnable(INT_TIMER0A); // enable interrupt on timer timeout
  ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0ISR);    // Registering  isr
  ROM_TimerEnable(TIMER0_BASE, TIMER_A); // enable the timer

}

// Initialize PWM module and digital output

/* void initPWM_DigOut(void) {

} */

//*****************************************************************************
//
// Main control loop that checks for the timer interrupt flag and determines the
// new control efforts based on encoder measurements
//
//*****************************************************************************


int
main(void)
{
    //volatile uint32_t ui32Loop;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2 & PF3).
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    // Initialize the UART.
    ConfigureUART();

    // Initialize the QEI module
    ConfigureQEI0();
    ConfigureQEI1();

    // Initialize timer for controller interrupts
    initTimer();

    // Initialize the PWM module and digitial outputs

    // Initialize the controller variables
    unsigned int Pos0 = 0;
    unsigned int Pos1 = 0;

    int Vel0 = 0;
    int Vel1 = 0;

    //int Spd0 = 0;
    //int Spd1 = 0;

    int U0 = 0;
    int U1 = 0;

    int k = 0;
    int b = 2;

    while(1)
    {
        // TODO: move all of this into an if() statement on flag set in the timer interrupt (start at ~10 Hz)
        if (controller_flag==1) {

          //
          // Turn on the LED.
          //
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

          //
          // Delay for a bit.
          //
          SysCtlDelay(SysCtlClockGet() / 3 / 1250000);

          //
          // Turn off the BLUE LED.
          //
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

          //
          // Delay for a bit.
          //
          SysCtlDelay(SysCtlClockGet() / 3 / 1250000);

          //
          // Send position and velocity values
          //
          Pos0 = QEIPositionGet(QEI0_BASE)/23;
          Pos1 = QEIPositionGet(QEI1_BASE)/23;

          Vel0 = QEIDirectionGet(QEI0_BASE)*QEIVelocityGet(QEI0_BASE);
          Vel1 = QEIDirectionGet(QEI1_BASE)*QEIVelocityGet(QEI1_BASE);

          //Spd0 = QEIVelocityGet(QEI0_BASE);
          //Spd1 = QEIVelocityGet(QEI1_BASE);

          U0 = (k*(Pos1-Pos0)) + (b*(Vel1-Vel0));
          U1 = (k*(Pos0-Pos1)) + (b*(Vel0-Vel1));

          // TODO: set new PWM values here and sync the PWM updates

          // TODO: make a variable to time this UART transmission, print that to console as well
          UARTprintf("%u, %d, %d, %u, %d, %d\n", Pos0, Vel0, U0, Pos1, Vel1, U1);

          // reset controller flag for next interrupt
          controller_flag = 0;

        }

    }
}
