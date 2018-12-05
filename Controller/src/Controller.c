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
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1); // E1 for module 0
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
    unsigned int Pos0raw = 0;
    unsigned int LastPos0raw = 0;
    unsigned int Pos1raw = 0;
    unsigned int LastPos1raw = 0;
    int DelPos0raw = 0;
    int DelPos1raw = 0;
    int PosDiffraw = 0;
    float PosDiffTemp = 0.0;
    float PosDiffFilt = 0.0;

    int Vel0raw = 0;
    int Vel1raw = 0;
    int VelOld0raw = 0;
    int VelOld1raw = 0;
    int VelDiffraw = 0;
    float VelDiffTemp = 0.0;
    float VelDiffFilt = 0.0;

    int U0 = 5000;
    int U1 = 5000;

    // values for ADC
    unsigned long ADCvals[4];
    unsigned long I0_raw = 0; // unconverted current values (still digital)
    unsigned long I1_raw = 0;
    float I0_real = 0.0;
    float I1_real = 0.0;

    // scaled values
    float ScaledPosDiff = 0;
    float ScaledVelDiff = 0;

    //int before = 0;
    //int after = 0;
    //int clock_time = 0;

    // contact controller values
    //float Kp_free = 10.0;
    //float Kp_contact = 100.0;
    //float Kp_alpha = 0.0;
    //float Kd = 45.0;
    //float Kd_B = 0.0;
    //float Imax = 1.5;
    //float Id = 0.0;
    //float alpha = 0.0;
    //int contact_flag = 0;
    //float filt_force = 0.0;
    //GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

    // vanilla controller values
    float Kp = 1.0; // Amps/rad
    float Kd = 0.0; // Amps/rad/sec
    float Kd_B = 0.0; // "bonus damping", same units as Kd_B
    float Imax = 1.5;
    float Id = 0.0;
    float Id_PD = 0.0;

    // SSI controller values
    float Id_SSI = 0.0;
    float Eout = 0.0;
    float Fslave = 0.0;
    float delO_new = 0.0;
    float delO = 0.0;
    float Kinc = 0.0;
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
    float Kv = 0.05; // initial guess, Kv = 2*bm/T
    float Ke = 1.0; // desired controller stiffness
    int SSI_flag = 1; // flag for whether SSI is activated
    int SSI_case = 0;

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
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);

    while(1)
    {
        // main controller to execute after each timer interrupt
        if (controller_flag==1) {

          // Turn on the LED. Can scope these pins for interrupt execution timing
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

          // Get position and velocity values
          Pos0raw = QEIPositionGet(QEI0_BASE); // divide by 23 to map to degrees
          Pos1raw = QEIPositionGet(QEI1_BASE);

          // Calculate control efforts, placing limits on values based on PWM period
          DelPos0raw = Pos0raw - LastPos0raw;
          if (DelPos0raw<-6144) { DelPos0raw = DelPos0raw + 8192; }
          if (DelPos0raw>6144)  { DelPos0raw = DelPos0raw - 8192; }
          DelPos1raw = Pos1raw - LastPos1raw;
          if (DelPos1raw<-6144) { DelPos1raw = DelPos1raw + 8192; }
          if (DelPos1raw>6144)  { DelPos1raw = DelPos1raw - 8192; }
          PosDiffraw = PosDiffraw + DelPos1raw - DelPos0raw; // calculate difference with 0 wrapping
          LastPos0raw = Pos0raw;
          LastPos1raw = Pos1raw;

          VelOld0raw = Vel0raw;
          VelOld1raw = Vel1raw;
          Vel0raw = DelPos0raw; // don't use QEI for velocities, remove scaling by time step
          Vel1raw = DelPos1raw; // scale by 1000/23 to get degrees per second

          //Acc0 = (Vel0 - VelOld0)*(2*M_PI/8.192)*1000.0; // in rad/sec/sec
          //Acc1 = (Vel1 - VelOld1)*(2*M_PI/8.192)*1000.0;

          // filter position
          PosDiffFilt = (0.0*PosDiffFilt) + (1.0*(float)PosDiffraw);
          PosDiffTemp = PosDiffFilt; // no deadzone for position

          // filter velocity
          VelDiffraw = Vel1raw-Vel0raw;
          VelDiffFilt = (0.0*VelDiffFilt) + (1.0*(float)VelDiffraw);
          VelDiffTemp = VelDiffFilt; // no deadzone for velocity

          ScaledPosDiff = PosDiffTemp*(2*M_PI/8192.0); // in rad
          ScaledVelDiff = VelDiffTemp*(2*M_PI/8.192); // approximately in rad/sec

          // PD calculations
          Id_PD = Kp*ScaledPosDiff + Kd*ScaledVelDiff;

          // SSI calculations
          if (SSI_flag==1) { // calculations for master 1, slave 0

            // determine SSI case 
            if (ScaledPosDiff>0.01) { // master is ahead of slave

              if (ScaledVelDiff<-0.01) { // slave is catching up to master (releasing cycle)
                Kinc = Kinc - Kv;
                SSI_case = 2; // second part of cycle (1st releasing)
              } else { // master is getting further ahead (pressing) or velocities are the same
                //delO = delO + Kv;
                if (ScaledVelDiff>0.01) {
                  Kinc = Kinc + Kv;
                  SSI_case = 1; // start of cycle (1st pressing)
                } else {
                  Kinc = Kinc;
                }
              }

            } else {
              if (ScaledPosDiff<-0.01) { // slave is ahead of master

                if (ScaledVelDiff>0.01) { // master is catching up to slave
                  Kinc = Kinc - Kv;
                  SSI_case = 4; // last part of cycle (2nd releasing)
                } else {  // slave is getting further ahead or velocities are the same
                  //delO = delO + Kv;
                  if (ScaledVelDiff<-0.01) {
                    Kinc = Kinc + Kv;
                    SSI_case = 3; // third part of cycle (2nd pressing)
                  } else {
                    Kinc = Kinc;
                  }
                }

              } else { // positions are the same
                Kinc = 0.0; // reset
                SSI_case = 0;
              }
            }
          } else {
            Kinc = 0.0;
          }

          // energy calculations based on SSI case
          if ((SSI_case==1)|(SSI_case==2)) {
            if (SSI_case==1) { // if pressing, check xtop and ftop
              if (ScaledPosDiff > xtop) {
                xtop = ScaledPosDiff;
              }
              if (Id_SSI > ftop) {
                ftop = Id_SSI;
              }
            } else { // if SSI_case==2, integrate energy
              Eout = Eout - (Ke*ScaledPosDiff*ScaledVelDiff*0.001);
            }
            delO_new = ((2*Eout)/xtop) - ((xtop*ftop)/2);
          }

          if (SSI_case==3) { // now on the input cycle, implement delO
            delO = delO_new; // update delO during case 3
            Eout = 0.0; // reset parameters
            xtop = 0.0;
            ftop = 0.0;
          }

          // set desired current, with offset depending on SSI case
          //Id_SSI = (Ke+Kinc)*ScaledPosDiff;
          if ((SSI_case==3)|(SSI_case==4)) { // if in second part of cycle, use most recently calculated delO
            if (ScaledVelDiff>0.01) {
              Id_SSI = Ke*ScaledPosDiff + delO;
            } else {
              if (ScaledVelDiff<-0.01) {
                Id_SSI = Ke*ScaledPosDiff - delO;
              } else {
                Id_SSI = Id_SSI; // no change in Id
              }
            }
          } else { // in first part of cycle (case 1 or 2), no offset yet
            Id_SSI = Ke*ScaledPosDiff; 
          }

          // contact-based controller calculations
          // calculate desired current based on tuning parameter alpha and pos/rate difference
          //sw_val = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
          //filt_force = 0.9*filt_force + 0.1*(float)I1_raw;

          /*if (filt_force>3000) { // && (contact_flag==0)) { // not in contact
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); // turn off LED if not in contact
            alpha = 1.0;
            Kp_alpha = Kp_alpha - 5.0; // integrate Kp to the free motion value
            if (Kp_alpha < 100.0) { Kp_alpha = 100.0; }
          }
          else if ((filt_force>3000) && (contact_flag==1)) { // recently left contact
            contact_flag = 0;
            Kp_alpha = Kp_alpha - 20.0; // integrate Kp to the free motion value
            if (Kp_alpha < 100.0) { Kp_alpha = 100.0; }
          }
          else if ((filt_force<3000) && (contact_flag==0)) { // recently entered contact
            contact_flag = 1;
            Kp_alpha = Kp_alpha + 100.0; // integrate Kp during contact to a new max value
            if (Kp_alpha > 1000.0) { Kp_alpha = 1000.0; }
          }
          else { // filt_force < 3000 and contact_flag==1 //  in contact
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // see timing of contact
            alpha = 0.0;
            Kp_alpha = Kp_alpha + 10.0; // integrate Kp during contact to a new max value
            if (Kp_alpha > 1000.0) { Kp_alpha = 1000.0; }
          }

          Kp_alpha = 100.0; */

          if (SSI_flag==1) {
            U0 = 5000 - (int)(Id_SSI*4000/Imax) - (int)(Kd_B*ScaledVelDiff*4000/Imax); // add bonus damping to the slave
            U1 = 5000 + (int)(Id_SSI*4000/Imax);
          } else {
            U0 = 5000 - (int)(Id_PD*4000/Imax) - (int)(Kd_B*ScaledVelDiff*4000/Imax); // add bonus damping to the slave
            U1 = 5000 + (int)(Id_PD*4000/Imax);
          }

          if (U0 < 1010) { U0 = 1010; } // limit to 15% and 85% for driver modules
          if (U0 > 8990) { U0 = 8990; }

          if (U1 < 1010) { U1 = 1010; }
          if (U1 > 8990) { U1 = 8990; }

          // set new PWM values
          PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, U0);
          PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, U1);

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
          //UARTprintf("%d, %d, %d, %d, %d, %d, %d, %d\n", SSI_case, U0-5000, (int)((float)Pos1raw*(2*M_PI/8.192)), (int)(ScaledPosDiff*1000), (int)(ScaledVelDiff*1000), (int)((Ke+Kinc)*1000), (int)(Id_SSI*1000), (int)(Id_PD*1000));
          UARTprintf("%d, %d, %d\n", SSI_case, (int)(delO*1000), (int)(Eout*1000));
          //after = TimerValueGet(TIMER0_BASE, TIMER_A);
          //transmit_time = before - after;

          // Turn off the LED.
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

          // Reset controller flag for next interrupt
          controller_flag = 0;

        }

    }
}
