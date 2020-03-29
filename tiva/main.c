#include "PLL.h"
#include "tm4c123gh6pm.h"

#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "driverlib/adc.h"
#include "driverlib/qei.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"
#include <string.h>
#include <stdio.h>
#include "inc/hw_timer.h"

//State observer math
#define HANDMADE_MATH_IMPLEMENTATION
#include "HandmadeMath.h"
#include "obsv.h"

/* -----------------------      Function Prototypes     --------------------- */

//Inits
void PortF_Init(void);
void PWM_Init(void);
void ADC_Init(void);
void UART_Init(void);
void QEI_Init(void);
void state_Init(void);
void Timer_Init(void);

//Interrupts, ISRs
void disable_interrupts(void);
void enable_interrupts(void);
void wait_for_interrupts(void);
void PortF_Handler(void);
void PortB_Handler(void);
void UARTIntHandler(void);

//Other
void MSDelay(unsigned int itime);
long read_ADC(void);
long movinAvg(void);
void send_u32(uint32_t n);
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void obsv(void);
void startTimer(void);
double stopTimer(void);

/* -----------------------      Global Variables        --------------------- */

uint32_t pui32ADC0Value[1]; //data from ADC0

volatile signed long dc = 49999; //0% duty cycle
volatile int theta_target = 0; //good starting guess
volatile int theta = 0;
volatile long pos = 0; //Cart position counter
volatile signed int pos_target = 10000; //Cart position counter
const float scaleTheta = (7.25*3.14159/4)/4096; //Convert Potentiometer to Radians (rads/counts)
const float scalePos = 0.05/1170; //Convert x pos to m (m/ticks)
volatile double dt = 0;

const int moving_avg_size = 10;
long thetas[moving_avg_size];

volatile bool run = false;

//State model variables
hmm_vec4 ABK1, ABK2, ABK3, ABK4, xHat, xHatNext, xHatDot, C1, C2, K, ref;
hmm_vec2 L1, L2, L3, L4, e, yHat, y;

void measureInputs(void){
    pos = QEIPositionGet(QEI0_BASE) - pos_target; //center at x = 0.
    theta = 4096 - movinAvg(); //flip theta to correspond with state model
}

/* -----------------------          Main Program        --------------------- */
int main(void){
    //Inits
    PLL_Init(); //set CPU clock to 80MHz
    PortF_Init();
    PWM_Init();
    ADC_Init();
    UART_Init();
    QEI_Init(); //Pins PD6,7 for quadrature encoder ch. A, B
    state_Init(); //Initialize state model matrices
    Timer_Init();

    // Master interrupt enable function for all interrupts
    IntMasterEnable();
    enable_interrupts();

    while(1){

        measureInputs();

        //Push SW1 to toggle run, ensure cart at middle of track.
        while(run & pos < 3000 & pos > -3000){

            measureInputs();

            //Update observer feedback
            obsv();

            //LQR Controller
            //xHat.X = pos*scalePos;
            //xHat.Z = (theta_target - theta)*scaleTheta;

            dc = -30000*HMM_DotVec4(xHatNext, K);
            //dc = 50*xHat.Z*K.Z; // just theta


            if(dc > 49999){
                dc = 49999;
            }
            else if(dc < -49999){
                dc = -49999;
            }


            if (dc > 0){
                PWM1_1_CMPA_R = 49999; //0% dc
                PWM1_1_CMPB_R = 50000 - dc;

            }
            else{
                PWM1_1_CMPA_R = 50000 + dc;
                PWM1_1_CMPB_R = 49999;
            }


        }
        //Stop motor when 'run' is false.
        PWM1_1_CMPB_R = 49999;
        PWM1_1_CMPA_R = 49999;

    }
}


/* Initialize PortF GPIOs */
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
    SYSCTL_RCGC2_R |= 0x00000020;           // activate clock for PortF
    while ((SYSCTL_PRGPIO_R & 0x00000020) == 0)
    {};                          // wait until PortF is ready
    GPIO_PORTF_LOCK_R = 0x4C4F434B;         // unlock GPIO PortF
    GPIO_PORTF_CR_R |= 0x1F;                 // allow changes to PF4-0
    GPIO_PORTF_AMSEL_R &= 0x00;              // disable analog on PortF
    GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
    GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
    GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
    GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
    GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4

    GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
    GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
    GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
    GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
    GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF)|0x00A00000; // (g) priority 5
    NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC for PF Handler
}

void QEI_Init(void){

    // Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);

    //DISable peripheral and int before configuration
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 20000);

    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);

    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI0_BASE, pos_target);
}

void PortF_Handler(void){

    MSDelay(800);//debounce
    GPIO_PORTF_ICR_R = 0x10;
    run = !run;
    if (run){
        //hold pendulum vertical when PF4 pushed to start controller and set target theta
        theta_target = theta;
        ref.Z = theta_target;
        //xHat = HMM_Vec4(pos_target, 0, theta_target, 0);
    }
}

void PortB_Handler(void){
    GPIO_PORTB_ICR_R = 0x80; //Clear interrupt flag

    if(GPIO_PORTB_DATA_R & 0x40){
        pos -= 1;
    }
    else{
        pos += 1;
    }

}

void PWM_Init(void) {
    SYSCTL_RCGCPWM_R |= 0x02;     // 1) enable PWM1 clock
    SYSCTL_RCGC2_R |= 0x10;          // 2.1) activate clock for PortE
    while ((SYSCTL_PRGPIO_R & 0x10) == 0) {}; // 2.2) wait until PortE is ready
    GPIO_PORTE_AFSEL_R |= 0x30;       // 3) enable alt function on PE5-4
    GPIO_PORTE_PCTL_R &= 0x00FF0000; // 4.1) clear PE5-4 GPIOPCTL PMCx fields
    GPIO_PORTE_PCTL_R |= 0x00550000; // 4.2) configure PE5-4 as PWM Module 1
    GPIO_PORTE_AMSEL_R &= ~0x30;     // 4.3) disable analog functionality on PE5-4
    GPIO_PORTE_DEN_R |= 0x30;        // 4.4) enable digital I/O on PE5-4
    SYSCTL_RCC_R |= 0x00100000;      // 5.1) configure PWM clock divider as the source for PWM clock
    SYSCTL_RCC_R &= ~0x000E0000;     // 5.2) clear PWMDIV field
    SYSCTL_RCC_R |= 0x000060000;     // 5.3) set divisor to 16 so PWM clock source is 1 MHz
    PWM1_1_CTL_R = 0;                // 6.1) disable PWM while initializing; also configure Count-Down mode
    PWM1_1_GENA_R = 0x08C;           // 6.2) drives pwmA HIGH when counter matches value in PWM1LOAD
                                          // drive pwmA LOW when counter matches comparator A
    PWM1_1_GENB_R = 0x80C;           // 6.3) drives pwmB HIGH when counter matches value in PWM1LOAD
                                          // drive pwmB LOW when counter matches comparator B
    PWM1_1_LOAD_R = 50000;
    PWM1_1_CMPA_R = 50000;
    PWM1_1_CMPB_R  = 50000;
    PWM1_1_CTL_R |= 0x01;            // 10) start the timers in PWM generator 1 by enabling the PWM clock
    PWM1_ENABLE_R |= 0x0C;           // 11) Enable M1PWM2 and M1PWM3
}

//Initialize console to display information while debugging
void UART_Init(void){

    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);


    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    //UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);


    // Configure the UART for 115,200, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    // Enable the UART interrupt.
    //
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);


    // Initialize the UART for console I/O, baud rate of 115,200
    //UARTStdioConfig(0, 115200, 16000000);
}


void MSDelay(unsigned int itime){
    unsigned int i;
    unsigned int j;
    for (i=0;i<itime; i++){
        for(j=0;j<331;j++){}
    }
}


void ADC_Init(void){
    //PB4 as analog input
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //enable ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //enable GPIO B
    GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4); //PB4 as analog input
    //PB6,7 as digital inputs

    //GPIO_PORTB_PUR_R |= 0xC0;     //     enable weak pull-up
    //GPIO_PORTB_PDR_R |= 0xC0;
    //GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    //GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    /*
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6); //PB6 as input
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_7); //PB7 as input
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD); //enable pull down resistors
    */

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 3.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH10 | ADC_CTL_IE |
                             ADC_CTL_END);

    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    ADCIntClear(ADC0_BASE, 3);

}

long read_ADC(void){
    //Trigger the conversion
    ADCProcessorTrigger(ADC0_BASE, 3);

    // Wait for conversion to be completed.
    while(!ADCIntStatus(ADC0_BASE, 3, false)){}

    // Clear the ADC interrupt flag.
    ADCIntClear(ADC0_BASE, 3);

    // Read ADC Value.
    ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
    return pui32ADC0Value[0];

    // Display the AIN10 (PB4) digital value on the console.
    //UARTprintf("PB4 = %4d\r", pui32ADC0Value[0],"\n");
}

long movinAvg(void){
    //Variables

    int i = 0;
    long sum = 0;

    sum = 0;
    for(i = moving_avg_size-1; i > 0; --i){
        thetas[i] = thetas[i-1];
        sum += thetas[i];
    }
    thetas[0] = read_ADC();
    sum += thetas[0];
    return sum/moving_avg_size;

}

void send_u32(uint32_t n) {
    UARTCharPut(UART0_BASE, n & 0xFF);
    UARTCharPut(UART0_BASE, (n >> 8) & 0xFF);
    UARTCharPut(UART0_BASE, (n >> 16) & 0xFF);
    UARTCharPut(UART0_BASE, (n >> 24) & 0xFF);
    //UARTCharPut(UART0_BASE, '\r\n');
}


//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);

    }
}

void UARTIntHandler(void)
{
    uint32_t ui32Status;

    // Get the interrupt status.
    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ui32Status);


    //UARTSend((uint8_t *)"nah\r\n", 5);
    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {

        // Read the next character from the UART and write it back to the UART.

        UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));

    }
    send_u32(theta);
    send_u32(pos);
    send_u32(dc);

}


void state_Init(void){
    //STATE MODEL from MATLAB
    //Constant matrix A - BK
    ABK1 = HMM_Vec4(0.0f, 1.0f, 0.0f, 0.0f);
    ABK2 = HMM_Vec4(abk2[0], abk2[1], abk2[2], abk2[3]);
    ABK3 = HMM_Vec4(0.0f, 0.0f, 0.0f, 1.0f);
    ABK4 = HMM_Vec4(abk4[0], abk4[1], abk4[2], abk4[3]);

    K = HMM_Vec4(k[0], k[1], k[2], k[3]);

    C1 = HMM_Vec4(1, 0, 0, 0);
    C2 = HMM_Vec4(0, 0, 1, 0);

    L1 = HMM_Vec2(l1[0], l1[1]);
    L2 = HMM_Vec2(l2[0], l2[1]);
    L3 = HMM_Vec2(l3[0], l3[1]);
    L4 = HMM_Vec2(l4[0], l4[1]);


    //hmm_vec2 y = HMM_Vec2(1.0f, 2.0f);
    yHat = HMM_Vec2(0.0f, 0.0f);
    e = HMM_Vec2(0.0f, 0.0f);

    //Initial conditions of 0 error
    xHat = HMM_Vec4(0.0f, 0.0f, 0.0f, 0.0f);
    xHatNext = HMM_Vec4(0.0f, 0.0f, 0.0f, 0.0f);
    xHatDot = HMM_Vec4(0.0f, 0.0f, 0.0f, 0.0f);

    ref.X = 0;
    ref.Y = 0;
    ref.Z = 0; //gets set when SW1 pushed
    ref.W = 0;

}

void obsv(void){
    y.X = (pos - ref.X)*scalePos;
    y.Y = (theta - ref.Z)*scaleTheta;

    yHat.X = HMM_DotVec4(C1, xHat);
    yHat.Y = HMM_DotVec4(C2, xHat);

    e = HMM_SubtractVec2(y, yHat);

    xHatDot.X = HMM_DotVec4(ABK1, xHat) + HMM_DotVec2(L1, e);
    xHatDot.Y = HMM_DotVec4(ABK2, xHat) + HMM_DotVec2(L2, e);
    xHatDot.Z = HMM_DotVec4(ABK3, xHat) + HMM_DotVec2(L3, e);
    xHatDot.W = HMM_DotVec4(ABK4, xHat) + HMM_DotVec2(L4, e);


    dt = stopTimer();

    xHatNext = HMM_AddVec4(xHat, HMM_MultiplyVec4f(xHatDot, dt));

    xHat = xHatNext;
    startTimer();

}


void Timer_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    SysCtlPeripheralReset(SYSCTL_PERIPH_TIMER3);    //
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT_UP);
    TimerControlStall(TIMER3_BASE, TIMER_A, true) ;
}

void startTimer(void) {
    TimerDisable(TIMER3_BASE, TIMER_A) ;
    HWREG(TIMER3_BASE + TIMER_O_TAV) = 0;
    TimerLoadSet(TIMER3_BASE, TIMER_A,0xFFFFFFFF) ;
    TimerEnable(TIMER3_BASE, TIMER_A) ;
}

double stopTimer(void) {
    //IntDisable(INT_TIMER3A);
    //TimerIntDisable(TIMER3_BASE, TIMER_TIMA_TIMEOUT) ;
    //TimerDisable(TIMER3_BASE, TIMER_A) ;
    uint32_t count = TimerValueGet(TIMER3_BASE, TIMER_A) ;
    return ((double)count/(double)SysCtlClockGet());  // forcing  double just in case there's an issue w compiler
}



/* Disable interrupts by setting the I bit in the PRIMASK system register */
void disable_interrupts(void) {
    __asm("    CPSID  I\n"
          "    BX     LR");
}


/* Enable interrupts by clearing the I bit in the PRIMASK system register */
void enable_interrupts(void) {
    __asm("    CPSIE  I\n"
          "    BX     LR");
}


/* Enter low-power mode while waiting for interrupts */
void wait_for_interrupts(void) {
    __asm("    WFI\n"
          "    BX     LR");
}
