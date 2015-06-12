//////////////////////////////////////////////////////////
//              Wristband Project Firmware              //
//              Written by Carlos Hernandez             //
//              San Jose State University               //
//////////////////////////////////////////////////////////

#include <asf.h>

/* Define system clock source (See conf_clock.h) */

//Global struct for accessing registers
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
	
	//set main clock as source of master clock
	PMC->PMC_MCKR = PMC_MCKR_CSS_MAIN_CLK;
	while(!(PMC->PMC_SR & PMC_SR_MCKRDY)){}
}

void enable_pio_clk(void){
		
	//Enable PIOA clock (ID: 11)
	PMC->PMC_PCER0 = PMC_PCER0_PID11;
		
	//Enable PIOB clock (ID: 12)
	PMC->PMC_PCER0 = PMC_PCER0_PID12;	
}

void enable_tc_clk(void){

	//enable TC clock (ID: 23) in PMC
	PMC->PMC_PCER0 = PMC_PCER0_PID23;
}
void pio_init(void){

	enable_pio_clk();
	//enable pin PA6 as output (LED)
	pioa_ptr->PIO_OER = PIO_PA6;
	
	//set LED pin PA6 as low (LED is active low)
//	pioa_regs->PIO_CODR = PIO_PA6;

	//set LED pin PA6 as high (LED is active low)
	pioa_ptr->PIO_SODR = PIO_PA6;
}

void tc_init(void){
		
	//set peripheral function for tc (B function) on pins PA0 & PA1
	pioa_ptr->PIO_ABCDSR[0] = PIO_ABCDSR_P0 | PIO_ABCDSR_P1;
	pioa_ptr->PIO_ABCDSR[1] = 0x00 | (0x00 << 1);
	
	//disable PIO control of PA0 & PA1 so TC can control pins
	pioa_ptr->PIO_PDR |= PIO_PA0 | PIO_PA1;
	
	enable_tc_clk();
	
	/*TC0 Setup*/
	//temporarily disable TC clk
	tc_ptr->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
		
	//set wave mode and to reset on match
	//also set/clear muxed pin to toggle on RA & RC match (scope debugging)
	tc_ptr->TC_CHANNEL[0].TC_CMR = 
		TC_CMR_WAVE
		| TC_CMR_WAVSEL_UP_RC 
		| TC_CMR_TCCLKS_TIMER_CLOCK4 // = 8Mhz * (1/128)
		| TC_CMR_ACPA_SET
		| TC_CMR_ACPC_CLEAR
		| TC_CMR_BCPB_CLEAR
		| TC_CMR_BCPC_SET;
	
	//set period & duty cycle 
	tc_ptr->TC_CHANNEL[0].TC_RA = 0x7A12; //duty cycle for TIOA
	tc_ptr->TC_CHANNEL[0].TC_RB = 0x7A12; //duty cycle for TIOB
	tc_ptr->TC_CHANNEL[0].TC_RC = 0xF424; //period (for TIOA & TIOB)
	
	//enable interrupt on compare match
	tc_ptr->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
	
	//enable tc clock
	tc_ptr->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;	
	/*End TC0 Setup*/	
	
	//Enable Interrupt in NVIC
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn, 0);
	NVIC_EnableIRQ(TC0_IRQn);
}


