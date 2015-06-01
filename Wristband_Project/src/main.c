//////////////////////////////////////////////////////////
//              Wristband Project Firmware              //
//              Written by Carlos Hernandez             //
//              San Jose State University               //
//////////////////////////////////////////////////////////

#include <asf.h>

/* Define system clock source (See conf_clock.h) */

//Global struct for accessing registers
Pio *pioa_regs = PIOA;
TcChannel *tc_regs = TC0;

//function prototypes
void enable_pio_clocks(void);
void pio_init(void);
void tc_init(void);

void TC0_Handler(void){
	
	//toggle LED @ TC0 Frequency
	if(pioa_regs->PIO_OSR & PIO_PA6){
		pioa_regs->PIO_CODR = PIO_PA6;
	}
	else{
		pioa_regs->PIO_SODR = PIO_PA6;
	}
	
	
}
//main function
int main (void)
{
	//Init. system clock
	sysclk_init();	
	pio_init();
	tc_init();
	
	while (1) {}
}

void pio_init(void){
	
	//Enable PIOA clock (ID: 11)
	PMC->PMC_PCER0 = PMC_PCER0_PID11;
	
	//Enable PIOA clock (ID: 12)
	PMC->PMC_PCER0 = PMC_PCER0_PID12;	
	
	//enable pin PA6 as output (LED)
	pioa_regs->PIO_OER = PIO_PA6;
	
	//set LED pin PA6 as low (LED is active low)
//	pioa_regs->PIO_CODR = PIO_PA6;

	//set LED pin PA6 as high (LED is active low)
//	pioa_regs->PIO_SODR = PIO_PA6;
}

void tc_init(void){
	
	//enable pin for tc periph
	pioa_regs->PIO_PDR = PIO_PA0;
		
	/*Clock setup*/
	//Temporarily disable PMC clock for peripheral (ID: 23)
// 	PMC->PMC_PCDR0 = PMC_PCER0_PID23;
//	PMC->PMC_PCR = PMC_PCR_CMD | PMC_PCR_PID(23) | PMC_PCR_DIV_PERIPH_DIV_MCK;
	//enable TC clock (ID: 23)
	PMC->PMC_PCER0 = PMC_PCER0_PID23;
	/*End Clock setup*/
	
	/*TC0 Setup*/
	//temporarily disable TC clk
	tc_regs->TC_CCR = TC_CCR_CLKDIS;
		
	//set wave mode and to reset on match
	tc_regs->TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
	
	//set compare number
	tc_regs->TC_RC = 0xFF;
	
	//enable interrupt on compare match
	tc_regs->TC_IER = TC_IER_CPCS;	
	/*End TC0 Setup*/	
	
	//Enable Interrupt in NVIC
	NVIC_SetPriority(TC0_IRQn, 0);
	NVIC_EnableIRQ(TC0_IRQn);
	
}