#include "SysTickInts.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"

void PortF_Init(void);
void PortE_PWM_Init(void);
void PWM_Init(void);
void PortF_Handler(void);
void MSDelay(unsigned int itime);
void disable_interrupts(void);
void enable_interrupts(void);
void wait_for_interrupts(void);

volatile unsigned long dc = 0;


/* main */
int main(void){
  PLL_Init();
  /*
  SYSCTL_RCC_R = 0x01D60D51;
      while((SYSCTL_RIS_R & SYSCTL_RIS_PLLLRIS)==0){};
      SYSCTL_RCC_R &= ~0x00000800;
      SYSCTL_RCC2_R = 0x00000000;
      */
  PortF_Init();
  PortE_PWM_Init();
  PWM_Init();

  enable_interrupts();
  while(1){
          wait_for_interrupts();
  }
}


/* Initialize PortF GPIOs */
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
    SYSCTL_RCGC2_R |= 0x00000020;           // activate clock for PortF
    while ((SYSCTL_PRGPIO_R & 0x00000020) == 0)
    {};                          // wait until PortF is ready
    GPIO_PORTF_LOCK_R = 0x4C4F434B;         // unlock GPIO PortF
    GPIO_PORTF_CR_R = 0x1F;                 // allow changes to PF4-0
    GPIO_PORTF_AMSEL_R = 0x00;              // disable analog on PortF
    GPIO_PORTF_PCTL_R = 0x00000000;         // use PF4-0 as GPIO
    GPIO_PORTF_DIR_R = 0x0E;                // PF4,PF0 in, PF3-1 out
    GPIO_PORTF_AFSEL_R = 0x00;              // disable alt function on PF
    GPIO_PORTF_PUR_R = 0x11;                // enable pull-up on PF0,PF4
    GPIO_PORTF_DEN_R = 0x1F;                // enable digital I/O on PF4-0

    GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
    GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
    GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
    GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
    GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF)|0x00A00000; // (g) priority 5
    NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}


void PWM_Init(void){
    SYSCTL_RCGC0_R |= 0x00100000; //activate PWM clock
    //SYSCTL_RCGCPWM_R |= 1; //enable clock to PWM0
    SYSCTL_RCC_R |= 0x00180000;
    SYSCTL_RCC_R &= ~0x00080000;
    PWM0_0_CTL_R = 0x00000000;
    PWM0_0_GENA_R = 0x0000008C;
    PWM0_0_LOAD_R = 0x7A12;
    PWM0_0_CMPA_R = 0x7A12;
    PWM0_0_CTL_R = 0x00000001;
    PWM0_ENABLE_R = 0x00000001;
}

void PortE_PWM_Init(void){
    /*SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE; //GPIO Port E PWM Clock enabled
    SYSCTL_RCGCGPIO_R |= 0x10;//enable clock to port E
    GPIO_PORTE_AFSEL_R |= 0x0030; //PE4,5 alternate functions enabled
    GPIO_PORTE_PCTL_R &= ~0x00FF0000;//make PE4,5 pwm outputs
    GPIO_PORTE_PCTL_R |= 0x00440000;//assigning pe4,5 to M0PWM4,5
    GPIO_PORTE_DEN_R |= 0x30;*/
    //Port E
    /*
    SYSCTL_RCGCGPIO_R |= 0x10;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;
    while((SYSCTL_PRGPIO_R & 0x00000010) == 0) {};
    GPIO_PORTE_LOCK_R = 0x4C4F434B;
    GPIO_PORTE_CR_R = 0x30;
    GPIO_PORTE_DIR_R |= 0x30;
    GPIO_PORTE_AFSEL_R |= 0x30;
    GPIO_PORTE_PCTL_R = 0x00440000;
    GPIO_PORTE_DEN_R |= 0x30;
    */
    //Port B
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGC2_R |= 0x00000002;
    while((SYSCTL_PRGPIO_R & 0x00000002) == 0) {};
    GPIO_PORTB_LOCK_R = 0x4C4F434B;
    GPIO_PORTB_CR_R = 0x40;
    GPIO_PORTB_DIR_R |= 0x40;
    GPIO_PORTB_AFSEL_R |= 0x40;
    GPIO_PORTB_PCTL_R = 0x04000000;
    GPIO_PORTB_DEN_R |= 0x40;

}

void MSDelay(unsigned int itime){
    unsigned int i;
    unsigned int j;
    for (i=0;i<itime; i++){
        for(j=0;j<331;j++){}
    }
}

void PortF_Handler(void){
    MSDelay (100);
    GPIO_PORTF_ICR_R = 0x10;
    PWM0_0_CMPA_R -= 0xC35;
    if (PWM0_0_CMPA_R <= 0) {
        PWM0_0_CMPA_R = 0x7A12;
    }

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
