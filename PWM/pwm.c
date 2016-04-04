/*
Runs on TM4C1294, Diego Aguirre, 04/12/2015

Title: blinky leds
Description:...
*/


// The #define statement PSYSDIV in PLL.h
// initializes the PLL to the desired frequency.
#include "PLL.h"
#include "sysTick_Init.h"
#include "tm4c1294ncpdt.h"

volatile unsigned long duty=5000;
int flag=0;

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode


void delayms(unsigned long ms);
void switch_Init(void);  
void pll_Init(void);
void pwm_Init(unsigned long period, unsigned long duty);
void SysTick_Init(void);
//void SysTick_Wait(unsigned long delay);
//void SysTick_Wait10ms(unsigned long delay);

void GPIOPortJ_Handler(void){//called on touch of either sw1 or sw2
  
	if(GPIO_PORTJ_RIS_R&0x02){// sw2 Touch
		GPIO_PORTJ_ICR_R = 0x02;// acknowledge flag1
		if(duty>1200) duty = duty-200; //slow down
	}
	if(GPIO_PORTJ_RIS_R&0x01){  // SW1 touch
		GPIO_PORTJ_ICR_R = 0x01;  // acknowledge flag0
		if(duty == 5000) duty = 2000;
    else if(duty<5000) duty = duty+200;   // speed up
  }
		
	flag = 1; 
}

/*void SysTick_Handler(void){
  if(GPIO_PORTA_DATA_R&0x20){   // toggle PA5
    GPIO_PORTA_DATA_R &= ~0x20; // make PA5 low
    NVIC_ST_RELOAD_R = L-1;     // reload value for low phase
  } else{
    GPIO_PORTA_DATA_R |= 0x20;  // make PA5 high
    NVIC_ST_RELOAD_R = H-1;     // reload value for high phase
  }
}*/

int main(void){
	DisableInterrupts();  // disable interrupts while initializing
  pll_Init();           // bus clock at 80 MHz
  switch_Init();        // arm PF4, PF0 for falling edge interrupts
  pwm_Init(0x8E07,duty);
	SysTick_Init();
	
	
  while(1){ // main program is free to perform other tasks
		if (flag ==1){
			pwm_Init(0x8E07,duty);
			flag = 0; //every time I press any buttom PWM on PF0 gets uptdated
		}
		else
			WaitForInterrupt(); // low power mode
  }
	
}


void switch_Init(void){
	unsigned long volatile delay;
	SYSCTL_RCGCGPIO_R  |= 0x100; //Port J clock
	delay=SYSCTL_RCGCGPIO_R;
	 GPIO_PORTJ_DIR_R  &= ~0x03; //PJ0 and PJ1 input
	GPIO_PORTJ_AFSEL_R  &= ~0x03; //no alternate function on PJ1
	GPIO_PORTJ_AMSEL_R &= ~0x03;  //no analog 
	GPIO_PORTJ_PCTL_R &= ~0x000000FF; //clear bits for PJ0 and PJ1
	//GPIO_PORTJ_DEN_R  |= 0x30; //enable PA4,PA2
	GPIO_PORTJ_DEN_R  |= 0x03; //enable PJ0and PJ1
	//GPIO_PORTJ_DATA_R|=0x10; //this is only when I have outputs
	GPIO_PORTJ_PUR_R  |= 0x03; //enable weak pull up on PJ0 and PJ1   
	GPIO_PORTJ_IS_R  &= ~0x03; // clear PJ0 and PJ1 to detect edge        
	GPIO_PORTJ_IBE_R &= ~0x03; //interrupts aren't caused by both edges on PJ0,1      
	GPIO_PORTJ_IEV_R &= ~0x03; //interrupts are only caused by a falling edge on PJ0,1     
	GPIO_PORTJ_IM_R  |= 0x03;   //arm interrupt on PJ0 and PJ1   
	GPIO_PORTJ_ICR_R = 0x03;  //clear flag pj0  
	NVIC_EN1_R |= 0x00080000;  //IRQ 51 bit 19 EN1
	NVIC_PRI12_R = (NVIC_PRI12_R&0x0FFFFFFF) | 0x40000000; //priority 2
	EnableInterrupts();  
}



void pwm_Init(unsigned long period, unsigned long duty){
	unsigned long volatile delay;
	SYSCTL_RCGCPWM_R |=0x01; //activate clock for PWM
	SYSCTL_RCGCGPIO_R  |= 0x20; //Port F clock
	delay=SYSCTL_RCGCGPIO_R;
	GPIO_PORTF_AFSEL_R |= 0x01;  // 3) enable alt funct on PF0
  GPIO_PORTF_DEN_R |= 0x01;   //    enable digital I/O on PF0                                     
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFFFF0)+0x00000006;// 4) configure PF0 as PWM0. We write a 6 
  GPIO_PORTF_AMSEL_R &= ~0x01;          //    disable analog functionality on PF0
	while((SYSCTL_PRPWM_R&SYSCTL_PRPWM_R0) == 0){}; //allow time for clock to stabilize
	PWM0_CC_R |= 0x100;           // 5) use PWM divider
  PWM0_CC_R &= ~0x07;       //clear PWM divider field
  PWM0_CC_R += 0x2;         //    configure for /8 divider
  PWM0_0_CTL_R = 0;                     // 6) re-loading down-counting mode
  PWM0_0_GENA_R = 0x0000008C;
  // PF0 goes hihg on LOAD
  // PF0 goes low on CMPA down
  PWM0_0_LOAD_R = period;           // 7) cycles needed to count down to 0
  PWM0_0_CMPA_R = period-duty-1;             // 8) count value when output rises
  PWM0_0_CTL_R |= 0x01;     // 9) start PWM0
  PWM0_ENABLE_R |= 0x01;  // 10) enable PWM0A/PF0 outputs
}

/* divider clock CC_R
/2-----0x0
/4-----0x1
/8-----0x2
/16----0x3
/32----0x4
/64----0x5

*/


void pll_Init(void){
	unsigned long timeout;
  // 1) Once POR has completed, the PIOSC is acting as the system clock.  Just in case
  //    this function has been called previously, be sure that the system is not being
  //    clocked from the PLL while the PLL is being reconfigured.
	//		bit 28 cleared(clk source is specified by OSCSRC field
  SYSCTL_RSCLKCFG_R &= ~SYSCTL_RSCLKCFG_USEPLL;
  // 2) Power up the MOSC by clearing the NOXTAL bit in the SYSCTL_MOSCCTL_R register.
  // 3) Since crystal mode is required, clear the PWRDN bit.  The datasheet says to do
  //    these two operations in a single write access to SYSCTL_MOSCCTL_R.
  SYSCTL_MOSCCTL_R &= ~(SYSCTL_MOSCCTL_NOXTAL|SYSCTL_MOSCCTL_PWRDN);
  //    Wait for the MOSCPUPRIS bit to be set in the SYSCTL_RIS_R register, indicating
  //    that MOSC crystal mode is ready.
  while((SYSCTL_RIS_R&SYSCTL_RIS_MOSCPUPRIS)==0){};
  // 4) Set both the OSCSRC and PLLSRC fields to 0x3 in the SYSCTL_RSCLKCFG_R register
  //    at offset 0x0B0.
  //    Temporarily get run/sleep clock from 25 MHz main oscillator.
  SYSCTL_RSCLKCFG_R = (SYSCTL_RSCLKCFG_R&~SYSCTL_RSCLKCFG_OSCSRC_M)+SYSCTL_RSCLKCFG_OSCSRC_MOSC;
  //    PLL clock from main oscillator.
  SYSCTL_RSCLKCFG_R = (SYSCTL_RSCLKCFG_R&~SYSCTL_RSCLKCFG_PLLSRC_M)+SYSCTL_RSCLKCFG_PLLSRC_MOSC;
  // 5) If the application also requires the MOSC to be the deep-sleep clock source,
  //    then program the DSOSCSRC field in the SYSCTL_DSCLKCFG_R register to 0x3.
  //    Get deep-sleep clock from main oscillator (few examples use deep-sleep; optional).
  //SYSCTL_DSCLKCFG_R = (SYSCTL_DSCLKCFG_R&~SYSCTL_DSCLKCFG_DSOSCSRC_M)+SYSCTL_DSCLKCFG_DSOSCSRC_MOSC;
  // 6) Write the SYSCTL_PLLFREQ0_R and SYSCTL_PLLFREQ1_R registers with the values of
  //    Q, N, MINT, and MFRAC to configure the desired VCO frequency setting.
  //    ************
  //    The datasheet implies that the VCO frequency can go as high as 25.575 GHz
  //    with MINT=1023 and a 25 MHz crystal.  This is clearly unreasonable.  For lack
  //    of a recommended VCO frequency, this program sets Q, N, and MINT for a VCO
  //    frequency of 480 MHz with MFRAC=0 to reduce jitter.  To run at a frequency
  //    that is not an integer divisor of 480 MHz, change this section.
  //    fVC0 = (fXTAL/(Q + 1)/(N + 1))*(MINT + (MFRAC/1024))
  //    fVCO = 480,000,000 Hz (arbitrary, but presumably as small as needed)
#define FXTAL 25000000  // fixed, this crystal is soldered to the Connected Launchpad
#define Q            0
#define N            4  // chosen for reference frequency within 4 to 30 MHz
#define MINT        96  // 480,000,000 = (25,000,000/(0 + 1)/(4 + 1))*(96 + (0/1,024))
#define MFRAC        0  // zero to reduce jitter
  //    SysClk = fVCO / (PSYSDIV + 1)
#define SYSCLK (FXTAL/(Q+1)/(N+1))*(MINT+MFRAC/1024)/(PSYSDIV+1)
  SYSCTL_PLLFREQ0_R = (SYSCTL_PLLFREQ0_R&~SYSCTL_PLLFREQ0_MFRAC_M)+(MFRAC<<SYSCTL_PLLFREQ0_MFRAC_S) |
                      (SYSCTL_PLLFREQ0_R&~SYSCTL_PLLFREQ0_MINT_M)+(MINT<<SYSCTL_PLLFREQ0_MINT_S);
  SYSCTL_PLLFREQ1_R = (SYSCTL_PLLFREQ1_R&~SYSCTL_PLLFREQ1_Q_M)+(Q<<SYSCTL_PLLFREQ1_Q_S) |
                      (SYSCTL_PLLFREQ1_R&~SYSCTL_PLLFREQ1_N_M)+(N<<SYSCTL_PLLFREQ1_N_S);
  SYSCTL_PLLFREQ0_R |= SYSCTL_PLLFREQ0_PLLPWR;       // turn on power to PLL
  SYSCTL_RSCLKCFG_R |= SYSCTL_RSCLKCFG_NEWFREQ;      // lock in register changes
  // 7) Write the SYSCTL_MEMTIM0_R register to correspond to the new clock setting.
  //    ************
  //    Set the timing parameters to the main Flash and EEPROM memories, which
  //    depend on the system clock frequency.  See Table 5-12 in datasheet.
  if(SYSCLK < 16000000){
    // FBCHT/EBCHT = 0, FBCE/EBCE = 0, FWS/EWS = 0
    SYSCTL_MEMTIM0_R = (SYSCTL_MEMTIM0_R&~0x03EF03EF) + (0x0<<22) + (0x0<<21) + (0x0<<16) + (0x0<<6) + (0x0<<5) + (0x0);
  } else if(SYSCLK == 16000000){
    // FBCHT/EBCHT = 0, FBCE/EBCE = 1, FWS/EWS = 0
    SYSCTL_MEMTIM0_R = (SYSCTL_MEMTIM0_R&~0x03EF03EF) + (0x0<<22) + (0x1<<21) + (0x0<<16) + (0x0<<6) + (0x1<<5) + (0x0);
  } else if(SYSCLK <= 40000000){
    // FBCHT/EBCHT = 2, FBCE/EBCE = 0, FWS/EWS = 1
    SYSCTL_MEMTIM0_R = (SYSCTL_MEMTIM0_R&~0x03EF03EF) + (0x2<<22) + (0x0<<21) + (0x1<<16) + (0x2<<6) + (0x0<<5) + (0x1);
  } else if(SYSCLK <= 60000000){
    // FBCHT/EBCHT = 3, FBCE/EBCE = 0, FWS/EWS = 2
    SYSCTL_MEMTIM0_R = (SYSCTL_MEMTIM0_R&~0x03EF03EF) + (0x3<<22) + (0x0<<21) + (0x2<<16) + (0x3<<6) + (0x0<<5) + (0x2);
  } else if(SYSCLK <= 80000000){
    // FBCHT/EBCHT = 4, FBCE/EBCE = 0, FWS/EWS = 3
    SYSCTL_MEMTIM0_R = (SYSCTL_MEMTIM0_R&~0x03EF03EF) + (0x4<<22) + (0x0<<21) + (0x3<<16) + (0x4<<6) + (0x0<<5) + (0x3);
  } else if(SYSCLK <= 100000000){
    // FBCHT/EBCHT = 5, FBCE/EBCE = 0, FWS/EWS = 4
    SYSCTL_MEMTIM0_R = (SYSCTL_MEMTIM0_R&~0x03EF03EF) + (0x5<<22) + (0x0<<21) + (0x4<<16) + (0x5<<6) + (0x0<<5) + (0x4);
  } else if(SYSCLK <= 120000000){
    // FBCHT/EBCHT = 6, FBCE/EBCE = 0, FWS/EWS = 5
    SYSCTL_MEMTIM0_R = (SYSCTL_MEMTIM0_R&~0x03EF03EF) + (0x6<<22) + (0x0<<21) + (0x5<<16) + (0x6<<6) + (0x0<<5) + (0x5);
  } else{
    // A setting is invalid, and the PLL cannot clock the system faster than 120 MHz.
    // Skip the rest of the initialization, leaving the system clocked from the MOSC,
    // which is a 25 MHz crystal.
    return;
  }
  // 8) Wait for the SYSCTL_PLLSTAT_R register to indicate that the PLL has reached
  //    lock at the new operating point (or that a timeout period has passed and lock
  //    has failed, in which case an error condition exists and this sequence is
  //    abandoned and error processing is initiated).
  timeout = 0;
  while(((SYSCTL_PLLSTAT_R&SYSCTL_PLLSTAT_LOCK) == 0) && (timeout < 0xFFFF)){
    timeout = timeout + 1;
  }
  if(timeout == 0xFFFF){
    // The PLL never locked or is not powered.
    // Skip the rest of the initialization, leaving the system clocked from the MOSC,
    // which is a 25 MHz crystal.
    return;
  }
  // 9)Write the SYSCTL_RSCLKCFG_R register's PSYSDIV value, set the USEPLL bit to
  //   enabled, and set the MEMTIMU bit.
  SYSCTL_RSCLKCFG_R = (SYSCTL_RSCLKCFG_R&~SYSCTL_RSCLKCFG_PSYSDIV_M)+(PSYSDIV&SYSCTL_RSCLKCFG_PSYSDIV_M) |
                       SYSCTL_RSCLKCFG_MEMTIMU |
                       SYSCTL_RSCLKCFG_USEPLL;
}


/*
PSYSDIV  SysClk (Hz)
  3     120,000,000
  4      96,000,000
  5      80,000,000
  7      60,000,000
  9      48,000,000
 15      30,000,000
 19      24,000,000
 29      16,000,000
 39      12,000,000
 79       6,000,000
*/

// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
	//NVIC_ST_RELOAD_R = 1599;
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
                                        // enable SysTick with core clock
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
}
// Time delay using busy wait.
// The delay parameter is in units of the core clock. (units of 8.333 nsec for 120 MHz clock)
void SysTick_Wait(unsigned long delay){
  volatile unsigned long elapsedTime;
  unsigned long startTime = NVIC_ST_CURRENT_R;
  do{
    elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);
}
// Time delay using busy wait.
// This assumes 80 MHz system clock.
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 1ms (assumes 80 MHz clock)
  }
}



