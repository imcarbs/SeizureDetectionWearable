//////////////////////////////////////////////////////////
//              Wristband Project Firmware              //
//              Written by Carlos Hernandez             //
//              San Jose State University               //
//////////////////////////////////////////////////////////

#include <asf.h>

/* Define system clock source (See conf_clock.h) */

//Global struct for accessing registers
Pio *pioa_regs = PIOA;

//function prototypes
void enable_pio_clocks(void);
void init_pio_ports(void);
void enable_tc_clocks(void);
void init_tc(void);

//main function
int main (void)
{
	//Init. system clock
	sysclk_init();
	enable_pio_clocks();
	init_pio_ports();

	while (1) {}
}

void enable_pio_clocks(void){
	
	//Enable PIOA clock (ID: 11)
	PMC->PMC_PCER0 = PMC_PCER0_PID11;
	
	//Enable PIOA clock (ID: 12)
	PMC->PMC_PCER0 = PMC_PCER0_PID12;
}

void init_pio_ports(void){
	
	//enable pin PA6 as output (LED)
	pioa_regs->PIO_OER = PIO_PA6;
	
	//set LED pin PA6 as low (LED is active low)
	pioa_regs->PIO_CODR = PIO_PA6;
}

void enable_tc_clocks(void){
	
	//Enable TC0 clock (ID: 23)
	PMC->PMC_PCER0 = PMC_PCER0_PID23;
	//edit
}