//////////////////////////////////////////////////////////
//              Wristband Project Firmware              //
//              Written by Carlos Hernandez             //
//              San Jose State University               //
//////////////////////////////////////////////////////////

#include <asf.h>

//Global structs for accessing registers
Pio *pioa_ptr = PIOA;
Pio *piob_ptr = PIOB;
Tc *tc_ptr = TC0;
Adc *adc_ptr = ADC;
Spi *spi_ptr = SPI0;
Flexcom *fc_ptr = FLEXCOM0;

int eda_voltage = 0;
int emg_voltage = 0;

//function prototypes
void enable_pio_clk(void);
void enable_tc_clk(void);
void enable_adc_clk(void);
void enable_i2c_clk(void);
void enable_spi_clk(void);
void pio_init(void);
void tc_init(void);
void adc_init(void);
void i2c_init(void);
void spi_init(void);
int voltage_to_adc(float voltage);

void TC0_Handler(void){
	
//check if button is pressed
//if(pioa_ptr->PIO_PDSR & PIO_PA2)	
	
	//get adc channel 0 & 1 current reading
	adc_ptr->ADC_CR = ADC_CR_START;
	eda_voltage = adc_ptr->ADC_CDR[0];
	emg_voltage = adc_ptr->ADC_CDR[1];
	
	//if ADC reads more than threshold, turn on buzzer
	if( (eda_voltage > voltage_to_adc(2.821)) || (emg_voltage > voltage_to_adc(0.101)) ){
			
		tc_ptr->TC_CHANNEL[0].TC_RB = 7500; // 50% duty cycle for TIOB (max buzzer volume)
		eda_voltage = 0;
		emg_voltage = 0;
	}
	
	else{
		tc_ptr->TC_CHANNEL[0].TC_RB = 0x0000; //0% duty cycle for TIOB (buzzer off)
		eda_voltage = 0;
		emg_voltage = 0;
	}
	
	//clear flag
	tc_ptr->TC_CHANNEL[0].TC_SR;
}

//main function
int main (void)
{
	//Init. system clock (PLLA aka 120 Mhz)
	//Uses EXTERNAL crystal oscillator
	//can be switched by redefining "CONFIG_SYSCLK_SOURCE"
	//see conf_clock.h for possible clock sources
	sysclk_init();
	pio_init();
	tc_init();
	adc_init();

	//empty while loop to run SAMG55 indefinitely
	while (1) {}
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

void enable_adc_clk(void){
	
	//enable ADC clock (ID: 29)
	PMC->PMC_PCER0 |= PMC_PCER0_PID29;
}

void enable_i2c_clk(void){
	
	//enable I2C0 clock (ID: 8)
	PMC->PMC_PCER0 |= PMC_PCER0_PID8;
}

void enable_spi_clk(void){
	
	/* SPI will use main clock as source
	b/c it is limited to 5 Mhz maximum */
	
	//disable PCK6 (for FLEXCOM0) to configure
	PMC->PMC_SCDR = PMC_SCDR_PCK6;
	
	//set main clock as source for PCK0 (8Mhz) scale to 2Mhz
	PMC->PMC_PCK[PMC_PCK_6] = PMC_PCK_CSS_MAIN_CLK | PMC_PCK_PRES_CLK_4;
	
	//enable PCK6
	PMC->PMC_SCER = PMC_SCER_PCK6;
	
	//wait for PCK6 to be ready
	while(!(PMC->PMC_SR & PMC_SR_PCKRDY6)){}

	//enable SPI0 (a.k.a. FLEXCOM0) clk (ID: 8)
	//PMC->PMC_PCER0 |= PMC_PCER0_PID8;	
}

void pio_init(void){

	enable_pio_clk();
	
	//enable pin PA6 as output (LED)
	pioa_ptr->PIO_OER |= PIO_PA6;
	
	//set LED pin PA6 as low (LED is active low)
//	pioa_regs->PIO_CODR = PIO_PA6;

	//set LED pin PA6 as high (LED is active low)
	pioa_ptr->PIO_SODR |= PIO_PA6;
	
	//enable switch (pull up resistor) [PA2]
	pioa_ptr->PIO_PUER |= PIO_PA2;
	//enable switch pin control by PIO [PA2]
	pioa_ptr->PIO_PER |= PIO_PA2;
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
		| TC_CMR_TCCLKS_TIMER_CLOCK1	// = 120Mhz (Master Clock) * (1/2)
		| TC_CMR_EEVT_XC0				//set xc0 so TIOB is not used as input
		| TC_CMR_ACPA_SET				//set PA0 on RA match
		| TC_CMR_ACPC_CLEAR				//clear PA0 on RC match
		| TC_CMR_BCPB_SET				//set PA1 on RB match
		| TC_CMR_BCPC_CLEAR;			//clear PA1 on RC match
	
	//set period & duty cycle (RC value = (120Mhz * prescaler)/(goal frequency))
	tc_ptr->TC_CHANNEL[0].TC_RA = 7500; //duty cycle for TIOA
	tc_ptr->TC_CHANNEL[0].TC_RB = 7500; //duty cycle for TIOB
	tc_ptr->TC_CHANNEL[0].TC_RC = 15000; //period (for TIOA & TIOB)
	
	//enable interrupt on RC compare match
	tc_ptr->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
	
	//enable tc clock & start tc
	tc_ptr->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;	
	/*End TC0 Setup*/	
	
	//Enable Interrupt in NVIC
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn, 0);
	NVIC_EnableIRQ(TC0_IRQn);
}

void adc_init(void){
	
	//set prescaler for ADC clk, track time to max.
	//set MR Transfer to 2 (default)
	adc_ptr->ADC_MR =
		ADC_MR_TRACKTIM(490)
		| ADC_MR_TRANSFER(2)
		| ADC_MR_PRESCAL(255);
	
	//enable ADC channel 0 (PA17 - EDA sensor)
	adc_ptr->ADC_CHER = ADC_CHER_CH0;
	
	//enable ADC channel 1 (PA18 - EMG sensor)
	adc_ptr->ADC_CHER = ADC_CHER_CH1;
	
	//enable ADC channel 2 (PA19 - Heart rate sensor)
//	adc_ptr->ADC_CHER = ADC_CHER_CH2;

	enable_adc_clk();	
}

void i2c_init(void){
	
}

void spi_init(void){
	
	//set peripheral function for SPI  (A function) on pin PA9 (MISO)
	pioa_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P9;
	pioa_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P9;

	//set peripheral function for SPI (A function) on pin PA10 (MOSI)
	pioa_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P10;
	pioa_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P10;	
	
	//set peripheral function for SPI (A function) on pin PB0 (SCK)
	piob_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P0;
	piob_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P0;	
		
 	//set peripheral function for SPI (A function) on pin PA25 (CS)
 	pioa_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P25;
 	pioa_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P25;
// 
// 	//set peripheral function for SPI (A function) on pin PA26 (RDYN)
// 	pioa_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P26;
// 	pioa_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P26;

	//disable PIO control of PA9, PB0 & PA10 so SPI0 can control pins
	pioa_ptr->PIO_PDR |= PIO_PA9;
	piob_ptr->PIO_PDR |= PIO_PB0;
	pioa_ptr->PIO_PDR |= PIO_PA10;
	pioa_ptr->PIO_PDR |= PIO_PA25;
//	pioa_ptr->PIO_PDR |= PIO_PA26;
	
	//enable pin PA25 as output (REQN)
//	pioa_ptr->PIO_OER |= PIO_PA25;
	
	//set pin PA25 to high (REQN active low)
//	pioa_ptr->PIO_SODR |= PIO_PA25;
	
// 	//enable pin PA26 as input (RDYN)
// 	pioa_ptr->PIO_PER |= PIO_PA26;
// 	
// 	//enable pin control by PIO (to use as RDYN) [PA26]
// 	pioa_ptr->PIO_PER |= PIO_PA26;
	
	//set flexcom mode to SPI
	fc_ptr->FLEXCOM_MR = FLEXCOM_MR_OPMODE_SPI;	
	
	//flexcom txdata register to hold SPI data (image)
	//fc_ptr->FLEXCOM_THR;

	//set as master, use peripheral clock
	spi_ptr->SPI_MR = SPI_MR_MSTR | SPI_MR_BRSRCCLK_PMC_PCK
					  | SPI_MR_PCS(1);
	//set to fixed mode
	spi_ptr->SPI_MR &= ~SPI_MR_PS;				  
	//set to direct connection to peripheral				  
	spi_ptr->SPI_MR &= ~SPI_MR_PCSDEC;
	
	//mode fault detection enable
	spi_ptr->SPI_MR &= ~SPI_MR_MODFDIS;
	
	//local loopback disabled
	spi_ptr->SPI_MR &= ~SPI_MR_LLB;
	
	//chip select settings
	spi_ptr->SPI_CSR[0] = SPI_CSR_CPOL		//CPOL = 1
						 | SPI_CSR_NCPHA	//NCPHA = 1
				         | SPI_CSR_BITS_8_BIT	//8-bit transfers
						 | SPI_CSR_CSNAAT	//chip select rise after transfer
						 | SPI_CSR_SCBR(2);	
						 
	//clear CSAAT (programmable clock source)
	spi_ptr->SPI_CSR[0] &= ~SPI_CSR_CSAAT;
	
	//enable SPI
	spi_ptr->SPI_CR = SPI_CR_SPIEN;	
	
	//sample write operation
	//spi_ptr->SPI_TDR = SPI_TDR_PCS(1) | SPI_TDR_TD(data)
	//				   | SPI_TDR_LASTXFER;
	
	enable_spi_clk();
}

int voltage_to_adc(float voltage){
	
	//assuming 3.3v is max. value
	//0v is min. value for adc
	//returns bit value for adc
	return ((int) (voltage * 4095)/3.3);
}