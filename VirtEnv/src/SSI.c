//*****************************************************************************
//
// Implementing SSI algorithm in a virtual environment on a 1DOF system
//
// Uses a Tiva TM4C123G Launchpad Evaluation Board
// 2 QEI modules, 1 timer, 2 PWM outputs, 2, digital outputs, 2 analog inputs, 1 UART port
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

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
#include "driverlib/fpu.h"

#include "utils/uartstdio.h"

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// define PI to test fpu
#ifndef M_PI
#define M_PI 3.14159265358979323846
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
  QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, 16000);

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

  // DON"T ENABLE MOTOR 0
  //GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1); // E1 for module 0

  // ENABLE MOTOR 1
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2); // E2 for module 1

  // analog inputs for actual motor current
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4); // set up AIN9 for motor 0
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5); // set up AIN8 for motor 1
  ADCSequenceDisable(ADC0_BASE, 1); // start config for sequence 1
  ADCReferenceSet(ADC0_BASE, ADC_REF_INT); // from "http://e2e.ti.com/support/microcontrollers/other_microcontrollers/f/908/t/301210" to fix saturation problem?
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
    FPUEnable();

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

    unsigned int Pos1raw = 0;
    unsigned int LastPos1raw = 0;
    int DelPos1raw = 0;

    float Pos1 = 0.0;
    float PosOld1 = 0.0;
    float Vel1 = 0.0;
    float VelOld1 = 0.0;

    int U1 = 5000;

    // values for ADC
    unsigned long ADCvals[4];
    unsigned long I0_raw = 0; // unconverted current values (still digital)
    unsigned long I1_raw = 0;
    float I0_real = 0.0;
    float I1_real = 0.0;

    // vanilla controller values
    float VEPos = 4.0;
    float Kp = 0.3; // Amps/rad
    float Imax = 1.5;
    float Id = 0.0;
    float IdOld = 0.0;
    int contact = 0;

    // SSI controller values
    float f = 0.0;
    float fe = 0.0;
    float fp = 0.0;
    float fr = 0.0;
    float x = 0.0;
    float xp = 0.0;
    float xtop = 0.0;
    float ftop = 0.0;
    float alpha = 0.0;
    float beta = 0.0;
    float mu = 0.0;
    float Kv = 0.15; // initial guess, Kv = 2*bm/T
    float Ke = 3; // desired stiffness
    int SSI_flag = 1; // flag for whether SSI is activated
    int pressing = 0; // flag for pressing or releasing cycles
    float Kdisp = 0.0; // calculate displayed stiffness

    // for dynamics calculations
    float kt = 19.4; // torque constant in mNm/A
    float J = 0.01454*0.075*0.075; // bolt "handle" inertia
    float Acc0 = 0.0;
    float Acc1 = 0.0;
    float Tm0 = 0.0;
    float Tin0 = 0.0;
    float Tm1 = 0.0;
    float Tin1 = 0.0;

    // Initialize timer for controller interrupts
    initTimer();

    // enable motor drivers just before entering control loop
    //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);

    while(1)
    {
        // main controller to execute after each timer interrupt
        if (controller_flag==1) {

          // Turn on the LED. Can scope these pins for interrupt execution timing
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

          // Get position and velocity
          Pos1raw = QEIPositionGet(QEI1_BASE);
          DelPos1raw = Pos1raw - LastPos1raw;
          if (DelPos1raw<-6144) { DelPos1raw = DelPos1raw + 8192; }
          if (DelPos1raw>6144)  { DelPos1raw = DelPos1raw - 8192; }
          LastPos1raw = Pos1raw;

          Pos1 = Pos1 + (float)DelPos1raw*(2*M_PI/8192.0); // in rad
          Vel1 = (float)DelPos1raw*(2*M_PI/8.192); // approximately in rad/sec
          Acc1 = (Vel1 - VelOld1)*(2*M_PI/8.192)*1000.0; // rad/s/s

          // filter velocity?

          // calculate control effort
          // SSI for VE
          x = Pos1-VEPos;

          if(x>0.0) { // if in contact with virtual wall

            fe = Ke*x;

            if(Vel1>0.0) { // in a pressing cycle
            //if(DelPos1raw>3) {

              if(pressing==0) { // newly started pressing cycle
                pressing = 1;
                alpha = Kv; //((fe-fp) / (fe - ((Kv*x)+fp-(Kv*xp)))); // calculate alpha for this cycle
                //UARTprintf("%d, %d, %d, %d, %d\n", (int)(fe), (int)(fp), (int)(x*1000), (int)(xp*1000), (int)(alpha*1000));

              }
              f = fp + Kv*(x-xp); //fe - ((fe-fp)/alpha);

              if(x>xtop) {xtop=x;}
              if(f>ftop) {ftop=f;}

            } else {
              if(Vel1<0.0) { // in a releasing cycle
              //if(DelPos1raw<-3) {
                if(pressing==1) { // newly started releasing cycles
                  pressing = 0;
                  mu = ftop/xtop; // use last f,x values from pressing cycle
                  ftop = 0.0;
                  xtop = 0.0;
                  beta = 0.1*Kv; //(((mu*x)-fp)/((Kv*x)-(Kv*xp)));
                }
                f = fp + beta*(x-xp); //fp + (((mu*x)-fp)/beta);
              } else { // velocity is 0, check pressing flag
                /*if(pressing==1) { // last in a pressing cycle
                  fe = Ke*x;
                  f = fe - ((fe-fp)/alpha);
                } else { // last in a releasing cycle
                  f = fp + (((mu*x)-fp)/beta);
                } */
                f = fp; // maintain force
              }
            }

            if (f>fe) {
              Id = fe; // set to desired stiffness
              Kdisp = Ke;
            } else {
              Id = f; // map force to desired current
              Kdisp = f/x;
            }
            contact = 1;

            // store previous values
            xp = x;
            fp = f;

          } else {
            Id = 0.0;
            contact = 0;
            xp = 0.0;
            fp = 0.0;
            f = 0.0;
            fe = 0.0;
            pressing = 0;
            Kdisp = 0.0;
          }

          if(SSI_flag==0) {
            Id = fe;
            Kdisp = Ke;
          }

          U1 = 5000 + (int)(Id*4000/Imax); // + delO;

          if (U1 < 1010) { U1 = 1010; }
          if (U1 > 8990) { U1 = 8990; }

          // set new PWM values
          //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, U0);
          PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, U1);

          // record values
          PosOld1 = Pos1;
          VelOld1 = Vel1;
          IdOld = Id;

          // sample motor current from analog inputs
          ADCIntClear(ADC0_BASE, 1);
          ADCProcessorTrigger(ADC0_BASE, 1);
          while(!ADCIntStatus(ADC0_BASE, 1, false)) {} // wait for sampling to finish
          ADCSequenceDataGet(ADC0_BASE, 1, ADCvals); // retrieve data from ADC FIFO buffer
          I0_raw = ADCvals[1]; // raw current value for motor 0
          I1_raw = ADCvals[0]; // raw current value for motor 1

          // calculate motor torques (in mNm) based on currents
          I0_real = ((((float)I0_raw)-1250.0)/1150.0)*Imax;
          I1_real = ((((float)I1_raw)-1250.0)/1150.0)*Imax;
          Tm0 = kt*I0_real;
          Tm1 = kt*I1_real;

          Tin0 = J*Acc0 - Tm0;
          Tin1 = J*Acc1 - Tm1;

          // transmit data
          //before = TimerValueGet(TIMER0_BASE, TIMER_A);
          UARTprintf("%d, %d, %d, %d, %d, %d\n", (int)(x*1000), (int)(f*1000), (int)(fe*1000), (int)(Id*1000), U1, (int)(Kdisp*1000));
          //UARTprintf("%d, %d, %d, %d\n", pressing, (int)(xtop*1000), (int)(ftop*1000), (int)(mu*1000));
          //after = TimerValueGet(TIMER0_BASE, TIMER_A);
          //transmit_time = before - after;

          // Turn off the LED.
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

          // Reset controller flag for next interrupt
          controller_flag = 0;

        }

    }
}
