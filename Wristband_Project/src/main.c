//////////////////////////////////////////////////////////
//              Wristband Project Firmware              //
//              Written by Carlos Hernandez             //
//              San Jose State University               //
//////////////////////////////////////////////////////////

#include <asf.h>

/* Define system clock source (See conf_clock.h) */

//Global structs for accessing registers
Pio *pioa_ptr = PIOA;
Tc *tc_ptr = TC0;

//function prototypes
void enable_pio_clk(void);
void enable_tc_clk(void);
void set_mck_source(void);
void pio_init(void);
void tc_init(void);

void TC0_Handler(void){
	
	//toggle LED @ TC0 Frequency
	//if PA6 is high, set to low
	if(pioa_ptr->PIO_PDSR & PIO_PA6){
		pioa_ptr->PIO_CODR = PIO_PA6;
	}
	//else set to high
	else{
		pioa_ptr->PIO_SODR = PIO_PA6;
	}
	
	//clear flag
	tc_ptr->TC_CHANNEL[0].TC_SR;
}

//main function
int main (void)
{
	//Init. system clock
	sysclk_init();
//	set_mck_source();
	pio_init();
	tc_init();
	
	while (1) {}
}

void set_mck_source(void){
	
	//set slow clock as source of master clock
	PMC->PMC_SCDR = PMC_SCER_PCK3;
	PMC->PMC_PCK[PMC_PCK_3] =
	(PMC->PMC_PCK[PMC_PCK_3] & ~PMC_PCK_CSS_Msk) | PMC_PCK_PRES_CLK_1;
	while ((PMC->PMC_SCER & (PMC_SCER_PCK0 << PMC_PCK_3))
	&& !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << PMC_PCK_3)));
	while(!(PMC->PMC_SR & PMC_SR_MCKRDY)){}
}

void enable_pio_clk(void){
		
	//Enable PIOA clock (ID: 11)
	PMC->PMC_PCER0 |= PMC_PCER0_PID11;
		
	//Enable PIOB clock (ID: 12)
	PMC->PMC_PCER0 |= PMC_PCER0_PID12;	
}

void enable_tc_clk(void){

	//enable TC clock (ID: 23) in PMC
	PMC->PMC_PCER0 |= PMC_PCER0_PID23;
}
void pio_init(void){

	enable_pio_clk();
	//enable pin PA6 as output (LED)
	pioa_ptr->PIO_OER |= PIO_PA6;
	
	//set LED pin PA6 as low (LED is active low)
//	pioa_regs->PIO_CODR = PIO_PA6;

	//set LED pin PA6 as high (LED is active low)
	pioa_ptr->PIO_SODR |= PIO_PA6;
}

void tc_init(void){
	
		
	//set peripheral function for tc (B function) on pin PA0
	pioa_ptr->PIO_ABCDSR[0] |= PIO_ABCDSR_P0;
	pioa_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P0;
	
	//set peripheral function for tc (B function) on pin PA1
	pioa_ptr->PIO_ABCDSR[0] |= PIO_ABCDSR_P1;
	pioa_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P1;
	
	//disable PIO control of PA0 & PA1 so TC can control pins
	pioa_ptr->PIO_PDR |= PIO_PA0;
	pioa_ptr->PIO_PDR |= PIO_PA1;
	
	//enable pmc periph clock for tc
	enable_tc_clk();
	
	/*TC0 Setup*/
	//temporarily disable TC clk input
	tc_ptr->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
		
	//set wave mode and to reset on RC match
	//also set muxed pin to toggle on RA & RC match (scope debugging)
	tc_ptr->TC_CHANNEL[0].TC_CMR = 
		TC_CMR_WAVE						//set TC for wave mode
		| TC_CMR_WAVSEL_UP_RC			//count up to RC value
		| TC_CMR_TCCLKS_TIMER_CLOCK4	// = 8Mhz (Master Clock) * (1/128)
		| TC_CMR_ACPA_SET				//set PA0 on RA match
		| TC_CMR_ACPC_CLEAR				//clear PA0 on RC match
		| TC_CMR_BCPB_SET				//set PA1 on RB match
		| TC_CMR_BCPC_CLEAR				//clear PA1 on RC match
		| TC_CMR_BSWTRG_CLEAR;			//set PA1 on software trigger
	
	//set period & duty cycle 
	tc_ptr->TC_CHANNEL[0].TC_RA = 0x0400; //duty cycle for TIOA
	tc_ptr->TC_CHANNEL[0].TC_RB = 0x0400; //duty cycle for TIOB
	tc_ptr->TC_CHANNEL[0].TC_RC = 0x0800; //period (for TIOA & TIOB)
	
	//enable interrupt on RC compare match
	tc_ptr->TC_CHANNEL[0].TC_IER = TC_IER_CPBS;
	
	//enable tc clock & start tc
	tc_ptr->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;	
	/*End TC0 Setup*/	
	
	//Enable Interrupt in NVIC
//	NVIC_DisableIRQ(TC0_IRQn);
//	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn, 0);
	NVIC_EnableIRQ(TC0_IRQn);
}


