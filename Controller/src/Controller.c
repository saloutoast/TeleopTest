//*****************************************************************************
//
// Simple controller for a 1-DOF master-slave teleoperation system
//
// Uses a Tiva TM4C123G Launchpad Evaluation Board
// 2 QEI modules, 1 timer, 2 PWM outputs, 2, digital outputs, 2 analog inputs, 1 UART port
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
#include "driverlib/adc.h"
#include "utils/uartstdio.h"

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

static volatile int controller_flag = 0;

// Configure the UART and its pins.  This must be called before UARTprintf().
void
ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O
    UARTStdioConfig(0, 460800, 16000000); // baud rate of 460800
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

	// Configure quadrature encoder
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

  //Set GPIO pins for QEI. PhA1 -> PC5, PhB1 ->PC6. I believe this sets the pull up and makes them inputs
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);

	//DISable peripheral and int before configuration
	QEIDisable(QEI1_BASE);
	QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

	// Configure quadrature encoder
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
  ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()/500); // set to roll over at 1kHz
  ROM_IntEnable(INT_TIMER0A); // enable interrupt on timer timeout
  ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0ISR);    // Registering  isr
  ROM_TimerEnable(TIMER0_BASE, TIMER_A); // enable the timer

}

// Initialize PWM module and digital output
void initMotorControl(void) {

  // PWM section
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

  SysCtlPWMClockSet(SYSCTL_PWMDIV_1); // clock predivider for PWM frequency

  GPIOPinConfigure(GPIO_PB6_M0PWM0);
  GPIOPinConfigure(GPIO_PB7_M0PWM1);

  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);

  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 10000); // PWM signal at 5KHz

  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 5000); // starting PWM duty cycles
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 5000);

  IntMasterEnable();

  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);

  // digital outputs to enable the driver boards
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1); // E1 for module 0
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2); // E2 for module 1

  // analog inputs for actual motor current
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4); // set up AIN9 for motor 0
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5); // set up AIN8 for motor 1
  ADCSequenceDisable(ADC0_BASE, 1); // start config for sequence 1
  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH8);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH9 |  ADC_CTL_IE | ADC_CTL_END );
  ADCSequenceEnable(ADC0_BASE, 1);
  ADCIntClear(ADC0_BASE, 1);
}


// Main control loop that checks for the timer interrupt flag and determines the
// new control efforts based on encoder measurements
int
main(void)
{
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPULazyStackingEnable();

    // Set the clocking to run directly from the crystal
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    // Enable the GPIO port that is used for the on-board LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable the GPIO pins for the LED (PF2 & PF3).
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    // Initialize the UART.
    ConfigureUART();

    // Initialize the QEI module
    ConfigureQEI0();
    ConfigureQEI1();

    // Initialize the PWM module and digitial outputs
    initMotorControl();

    // Initialize the controller variables
    unsigned int Pos0 = 0;
    unsigned int LastPos0 = 0;
    unsigned int Pos1 = 0;
    unsigned int LastPos1 = 0;
    int PosDiff = 0;
    int PosDiffTemp = 0;
    int PosDiffFilt = 0;

    int Vel0 = 0;
    int Vel1 = 0;
    int VelDiff = 0;
    int VelDiffTemp = 0;
    int VelDiffFilt = 0;

    int U0 = 0;
    int U1 = 0;

    int k = 20;
    int b = 10;

    // values for ADC
    unsigned long ADCvals[4];
    long I0_raw = 0; // unconverted current values (still digital)
    long I1_raw = 0;

    //int before = 0;
    //int after = 0;
    //int clock_time = 0;


    // Initialize timer for controller interrupts
    initTimer();

    // enable motor drivers just before entering control loop
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);

    while(1)
    {
        // main controller to execute after each timer interrupt
        if (controller_flag==1) {

          // Turn on the LED. Can scope these pins for interrupt execution timing
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

          // Get position and velocity values
          Pos0 = QEIPositionGet(QEI0_BASE); // divide by 23 to map to degrees
          Pos1 = QEIPositionGet(QEI1_BASE);

          Vel0 = QEIDirectionGet(QEI0_BASE)*QEIVelocityGet(QEI0_BASE);
          Vel1 = QEIDirectionGet(QEI1_BASE)*QEIVelocityGet(QEI1_BASE);

          // Calculate control efforts, placing limits on values based on PWM period
          PosDiff = PosDiff + (Pos1-LastPos1) - (Pos0-LastPos0); // calculate difference with 0 wrapping
          PosDiffFilt = ((3*PosDiffFilt) + PosDiff) / 4; // filter the positions
          LastPos0 = Pos0;
          LastPos1 = Pos1;
          //if ((Pos1<2048) & (Pos0>6144)) { PosDiff = Pos1 + 8192 - Pos0; }
          //if ((Pos0<2048) & (Pos1>6144)) { PosDiff = Pos0 + 8192 - Pos1; }

          if ((PosDiff<25) & (PosDiff>-25)) { PosDiffTemp = 0; }
          else { PosDiffTemp = PosDiff; }

          VelDiff = Vel1-Vel0;
          VelDiffFilt = ((9*VelDiffFilt) + VelDiff) / 10;

          if ((VelDiffFilt<25) & (VelDiffFilt>-25)) { VelDiffTemp = 0; }
          else { VelDiffTemp = VelDiffFilt; }

          U0 = 5000 - (k*(PosDiffTemp)) - (b*VelDiffTemp);
          U1 = 5000 + (k*(PosDiffTemp)) + (b*VelDiffTemp);

          if (U0 < 1010) { U0 = 1010; } // limit to 15% and 85% for driver modules
          if (U0 > 8990) { U0 = 8990; }

          if (U1 < 1010) { U1 = 1010; }
          if (U1 > 8990) { U1 = 8990; }

          // set new PWM values
          PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, U0);
          PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, U1);

          // sample motor current from analog inputs
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // see timing of sampling
          ADCIntClear(ADC0_BASE, 1);
          ADCProcessorTrigger(ADC0_BASE, 1);
          while(!ADCIntStatus(ADC0_BASE, 1, false)) {} // wait for sampling to finish
          ADCSequenceDataGet(ADC0_BASE, 1, ADCvals); // retrieve data from ADC FIFO buffer
          I0_raw = ADCvals[1]; // raw current value for motor 0
          I1_raw = ADCvals[0]; // raw current value for motor 1
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); // turn off LED after sampling process is complete

          // transmit data
          //before = TimerValueGet(TIMER0_BASE, TIMER_A);
          UARTprintf("%u, %d, %d, %d, %u, %d, %d, %d\n", Pos0, Vel0, U0, I0_raw, Pos1, Vel1, U1, I1_raw);
          //UARTprintf("%d, %d\n", I0_raw, I1_raw); // only return currents
          //after = TimerValueGet(TIMER0_BASE, TIMER_A);
          //transmit_time = before - after;

          // Turn off the LED.
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);


          // Reset controller flag for next interrupt
          controller_flag = 0;

        }

    }
}
