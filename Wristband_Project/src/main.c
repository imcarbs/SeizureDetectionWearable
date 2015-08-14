//////////////////////////////////////////////////////////
//              Wristband Project Firmware              //
//              Written by Carlos Hernandez             //
//              San Jose State University               //
//////////////////////////////////////////////////////////

#include <asf.h>

//Global structs for accessing registers
Pio *pioA_ptr = PIOA;
Pio *pioB_ptr = PIOB;
Tc *tc0_ptr = TC0;
Adc *adc_ptr = ADC;
Spi *spi0_ptr = SPI0;
Flexcom *fc0_ptr = FLEXCOM0;

//Global variables
uint32_t eda_voltage = 0;
uint32_t emg_voltage = 0;
uint32_t ecg_voltage = 0;
uint32_t spi_init_timer = 0;
volatile int spi_init_complete = 0;
volatile int spi_init_begin = 0;
volatile int adxl_data_ready = 0;
volatile int32_t x_Axis_data = 0;
volatile int32_t y_Axis_data = 0;
volatile int32_t z_Axis_data = 0;

//function prototypes
void enable_pio_clk(void);
void enable_tc_clk(void);
void enable_adc_clk(void);
void enable_i2c_clk(void);
void enable_spi0_clk(void);
void init_pio(void);
void init_tc(void);
void init_adc(void);
void init_i2c(void);
void init_adxl_spi0(void);
void init_adxl(void);
void verify_adxl_spi0(void);
void write_adxl(uint32_t address, uint32_t data);
int32_t read_adxl(uint32_t address, uint32_t data);
int32_t update_accel_data(void);
uint32_t voltage_to_adc(float voltage);

void TC0_Handler(void){
	
	//check if button is pressed
	//if(pioA_ptr->PIO_PDSR & PIO_PA2)	
	
	//toggle PA11 pin to check ISR timing
//	pioA_ptr->PIO_CODR |= PIO_CODR_P11;
	
	//get adc channels 0 & 1 current reading
	adc_ptr->ADC_CR = ADC_CR_START;
	eda_voltage = adc_ptr->ADC_CDR[0];
	emg_voltage = adc_ptr->ADC_CDR[1];
//	ecg_voltage = adc_ptr->ADC_CDR[2];
	
	//toggle PA11 pin to check ISR timing
//	pioA_ptr->PIO_CODR |= PIO_CODR_P11;	
	//get accel. data
	if (spi_init_complete == 1 && adxl_data_ready == 1){
		update_accel_data();
	}
	//toggle PA11 pin to check timing
//	pioA_ptr->PIO_SODR |= PIO_SODR_P11;
	
	//if ADC reads more than threshold, turn on buzzer
	if( (eda_voltage > voltage_to_adc(2.821)) 
		|| (emg_voltage > voltage_to_adc(0.101)) ){
		
		// 50% duty cycle on TIOB (max volume)	
		tc0_ptr->TC_CHANNEL[0].TC_RB = 7500; 
		eda_voltage = 0;
		emg_voltage = 0;
		//edc_voltage = 0;
	}
	
	else{
		
		//0% duty cycle on TIOB (buzzer off)
		tc0_ptr->TC_CHANNEL[0].TC_RB = 0x0000; 
		eda_voltage = 0;
		emg_voltage = 0;
		//edc_voltage = 0;
	}
	
	if(spi_init_timer != 100){
		spi_init_timer++;
	}
	
	else {
		spi_init_begin = 1;
	}
	
	//clear TC0 interrupt flags
	tc0_ptr->TC_CHANNEL[0].TC_SR;
	
	//toggle PA11 pin to check timing
//	pioA_ptr->PIO_SODR |= PIO_SODR_P11;
}

void PIOA_Handler(void){
	
	//PA12
}

void PIOB_Handler(void){

	//toggle PA11 pin to check ISR timing
//	pioA_ptr->PIO_CODR |= PIO_CODR_P11;
	
	//check interrupt source on PB pins
	if(pioB_ptr->PIO_ISR & PIO_ISR_P9){
		
		//Set ADXL data ready flag	
		adxl_data_ready = 1;
		//check interrupt source on ADXL
//		if((read_adxl(0x30, 0x00) & 0xF0) == 0x80){

//			pioA_ptr->PIO_CODR = PIO_PA6;
//		}

	}
	
	//clear ISR flag
	pioB_ptr->PIO_ISR;
	
	//toggle PA11 pin to check ISR timing
//	pioA_ptr->PIO_SODR |= PIO_SODR_P11;
}

//main function
int main (void)
{
	//Init. system clock (PLLA aka 120 Mhz)
	//Uses EXTERNAL crystal oscillator
	//can be switched by redefining "CONFIG_SYSCLK_SOURCE"
	//see conf_clock.h for possible clock sources
	sysclk_init();
	init_pio();
	init_tc();
	init_adc();
	init_adxl_spi0();
	init_adxl();
	
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

void enable_spi0_clk(void){
	
	/* SPI will use main clock as source
	b/c it is limited to 5 Mhz maximum */
	
	//enable SPI0 (a.k.a. FLEXCOM0) clk (ID: 8)
	PMC->PMC_PCER0 |= PMC_PCER0_PID8;	
	
	//disable PCK6 (for FLEXCOM0) to configure
	PMC->PMC_SCDR = PMC_SCDR_PCK6;
	
	//set main clock as source for PCK0 (8Mhz) scale to 8Mhz/4
	PMC->PMC_PCK[PMC_PCK_6] = PMC_PCK_CSS_MAIN_CLK | PMC_PCK_PRES_CLK_2;
	
	//enable PCK6
	PMC->PMC_SCER = PMC_SCER_PCK6;
	
	//wait for PCK6 to be ready
	while(!(PMC->PMC_SR & PMC_SR_PCKRDY6)){}
}

void init_pio(void){

	enable_pio_clk();
	
	//enable pin PA6 as output (LED)
	pioA_ptr->PIO_OER |= PIO_PA6;
	
	//enable pin PA11 as output (Debugging)
	pioA_ptr->PIO_OER |= PIO_PA11;

	//set LED pin PA6 as high (LED is active low)
	pioA_ptr->PIO_SODR |= PIO_PA6;
	
	//enable switch (pull up resistor) [PA2]
	pioA_ptr->PIO_PUER |= PIO_PA2;
	
	//enable switch pin control by PIO [PA2]
	pioA_ptr->PIO_PER |= PIO_PA2;
}

void init_tc(void){
		
	//set peripheral function for tc (B function) on pin PA0
	pioA_ptr->PIO_ABCDSR[0] |= PIO_ABCDSR_P0;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P0;
	
	//set peripheral function for tc (B function) on pin PA1
	pioA_ptr->PIO_ABCDSR[0] |= PIO_ABCDSR_P1;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P1;
	
	//disable PIO control of PA0 & PA1 so TC can control pins
	pioA_ptr->PIO_PDR |= PIO_PA0;
	pioA_ptr->PIO_PDR |= PIO_PA1;
	
	//enable pmc periph clock for tc
	enable_tc_clk();
	
	/*TC0 Setup*/
	//temporarily disable TC clk input
	tc0_ptr->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
		
	//set wave mode and to reset on RC match
	//also set muxed pin to toggle on RA & RC match (scope debugging)
	tc0_ptr->TC_CHANNEL[0].TC_CMR = 
		TC_CMR_WAVE						//set TC for wave mode
		| TC_CMR_WAVSEL_UP_RC			//count up to RC value
		| TC_CMR_TCCLKS_TIMER_CLOCK1	// = 120Mhz (Master Clock) * (1/2)
		| TC_CMR_EEVT_XC0				//set xc0 so TIOB is not used as input
		| TC_CMR_ACPA_SET				//set PA0 on RA match
		| TC_CMR_ACPC_CLEAR				//clear PA0 on RC match
		| TC_CMR_BCPB_SET				//set PA1 on RB match
		| TC_CMR_BCPC_CLEAR;			//clear PA1 on RC match
	
	//set period & duty cycle (RC value = (120Mhz * prescaler)/(goal frequency))
	tc0_ptr->TC_CHANNEL[0].TC_RA = 7500; //duty cycle for TIOA
	tc0_ptr->TC_CHANNEL[0].TC_RB = 7500; //duty cycle for TIOB
	tc0_ptr->TC_CHANNEL[0].TC_RC = 15000; //period (for TIOA & TIOB)
	
	//enable interrupt on RC compare match
	tc0_ptr->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
	
	//enable tc clock & start tc
	tc0_ptr->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;	
	/*End TC0 Setup*/	
	
	//Enable TC0 Interrupt in NVIC
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn, 0);
	NVIC_EnableIRQ(TC0_IRQn);
}

void init_adc(void){
	
	//set prescaler for ADC clk (max), track time (max)
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

void init_i2c(void){
	
}

void init_adxl_spi0(void){
	
	//disable SPI to configure
	spi0_ptr->SPI_CR = SPI_CR_SPIDIS;
	
	//reset SPI
	spi0_ptr->SPI_CR = SPI_CR_SWRST;
	
	//set peripheral function A for SPI on pin PA09 (MISO/SDO)
	pioA_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P9;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P9;

	//set peripheral function A for SPI on pin PA10 (MOSI/SDA)
	pioA_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P10;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P10;
	
	//set peripheral function A for SPI on pin PB00 (SPCK/SCL)
	pioB_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P0;
	pioB_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P0;
		
 	//set peripheral function A for SPI on pin PA25 (NPCS0/NSS/CS)
 	pioA_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P25;
 	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P25;
	 
	//enable PIO control of PB09 for use as input for INT1
	pioB_ptr->PIO_PER |= PIO_PB9;
	
	//disable output
	pioB_ptr->PIO_ODR |= PIO_PB9;
	
	//enable interrupt on PB09
	pioB_ptr->PIO_IER |= PIO_PB9;
	
	//enable additional interrupt settings for PB09
	pioB_ptr->PIO_AIMER |= PIO_PB9;
	
	//enable edge detection for PB09
	pioB_ptr->PIO_ESR |= PIO_PB9;
	
	//set to detect rising edge for PB09 (INT1)
	pioB_ptr->PIO_REHLSR |= PIO_PB9;
	
	//disable PIO control of PA09, PB00, PA25, & PA10
	//so SPI0 can control pins
	pioA_ptr->PIO_PDR |= PIO_PA9;
	pioB_ptr->PIO_PDR |= PIO_PB0;
	pioA_ptr->PIO_PDR |= PIO_PA10;
	pioA_ptr->PIO_PDR |= PIO_PA25;
	
	//set flexcom0 mode to SPI
	fc0_ptr->FLEXCOM_MR = FLEXCOM_MR_OPMODE_SPI;	
	
	//set as master, use peripheral clock, mode fault disable
	spi0_ptr->SPI_MR = SPI_MR_MSTR | SPI_MR_BRSRCCLK_PMC_PCK
					  | SPI_MR_PCS(0) | SPI_MR_MODFDIS;
					  
	//set to fixed peripheral mode
	spi0_ptr->SPI_MR &= ~SPI_MR_PS;			

	//set to direct connection to peripheral				  
	spi0_ptr->SPI_MR &= ~SPI_MR_PCSDEC;

	//local loopback disabled
	spi0_ptr->SPI_MR &= ~SPI_MR_LLB;
	
	//chip select settings
	spi0_ptr->SPI_CSR[0] = SPI_CSR_CPOL			//CPOL = 1
				         | SPI_CSR_BITS_16_BIT	//16-bit transfers
						 | SPI_CSR_SCBR(2)		//bit rate 1/2 pclk
						 | SPI_CSR_DLYBS(4)		//delay after cs before sck
						 | SPI_CSR_DLYBCT(0);	//0 delay btwn multibyte transfers
	
	//CPHA = 1 (NCPHA = 0)
	spi0_ptr->SPI_CSR[0] &= ~SPI_CSR_NCPHA;
					 
	//clear CSAAT (programmable clock source)
	spi0_ptr->SPI_CSR[0] &= ~SPI_CSR_CSAAT;
	
	//enable PIOB interrupt for data ready interrupt (INT1) in NVIC
	NVIC_DisableIRQ(PIOB_IRQn);
	NVIC_ClearPendingIRQ(PIOB_IRQn);
	NVIC_SetPriority(PIOB_IRQn, 1);
	NVIC_EnableIRQ(PIOB_IRQn);

	//next transfer is last
	spi0_ptr->SPI_CR = SPI_CR_LASTXFER;
	
	//wait for adxl to power up
	while(spi_init_begin == 0){}

	enable_spi0_clk();	
	
	//enable SPI
	spi0_ptr->SPI_CR = SPI_CR_SPIEN;
}

void init_adxl(void){

//	verify_adxl_spi0();

	//set to 13-bit mode, +-16g
	write_adxl(0x31, 0x0B);
	
	//set Rate to 1600Hz
	write_adxl(0x2C, 0x0E);
	
	//check to see if data rate is actually at 1600 Hz
// 	if((read_adxl(0x2C, 0x00) & 0x0F) == 0xE){
// 		
// 		pioA_ptr->PIO_CODR = PIO_PA6;
// 	}	
	
	//Start Measurement
	write_adxl(0x2D, 0x08);
	
	//Map data_ready interrupt to INT1 pin
	write_adxl(0x2F, 0x7F);
	
	//Enable data_ready interrupt
	write_adxl(0x2E, 0x80);
	
	//Set flag so TC0 ISR can update axis data
	spi_init_complete = 1;
}

//writes to single register on ADXL
void write_adxl(uint32_t address, uint32_t data){
	
	//combine data and address
	data |= (address << 8);
	
	//write to transfer data register
	spi0_ptr->SPI_TDR = SPI_TDR_TD(data);
		
	//wait for transaction to end
	while((spi0_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
		
	//read data register to clear flag
	data = spi0_ptr->SPI_RDR;
}

//reads single register from ADXL, returns 8 bit value read
int32_t read_adxl(uint32_t address, uint32_t data){
	
	//combine data and address, set read bit
	data |= (address << 8) | 0x8000;
		
	//write to transfer data register
	spi0_ptr->SPI_TDR = SPI_TDR_TD(data);
	
	//wait for transaction to end
	while((spi0_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
	
	//discard don't cares, return 8 bits
	return (spi0_ptr->SPI_RDR & 0xFF);
}

int32_t update_accel_data(void){
	
	uint32_t adxlData0 = 0;
	uint32_t adxlData1 = 0;
	uint32_t adxlData2 = 0;
	uint32_t adxlData3 = 0;
	
	//write to start data transfer
	//read command (R = 1) multibyte bit on (MB = 1), address 0x32
	spi0_ptr->SPI_TDR = SPI_TDR_TD(0xF200);
	
	//wait for data to load into shift register
	while(!(spi0_ptr->SPI_SR & SPI_SR_TDRE)){}
	
	//start loading next data
	spi0_ptr->SPI_TDR = SPI_TDR_TD(0xFFFF);
	
	//check if read is ready
	while((spi0_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
	
	//save data
	adxlData0 = spi0_ptr->SPI_RDR;
	
	//clear ready flag
	adxl_data_ready = 0;
	
	//wait for data to load into shift register
	while(!(spi0_ptr->SPI_SR & SPI_SR_TDRE)){}

	//start loading next data
	spi0_ptr->SPI_TDR = SPI_TDR_TD(0xFFFF);
	
	//check if read is ready
	while((spi0_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
	
	//save data
	adxlData1 = spi0_ptr->SPI_RDR;
	
	//wait for data to load into shift register
	while(!(spi0_ptr->SPI_SR & SPI_SR_TDRE)){}

	//start loading next data
	spi0_ptr->SPI_TDR = SPI_TDR_TD(0xFFFF);
	
	//check if read is ready
	while((spi0_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
	
	//save data
	adxlData2 = spi0_ptr->SPI_RDR;
	
	//check if read is ready
	while((spi0_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
	
	//save data
	adxlData3 = spi0_ptr->SPI_RDR;
	
	//merge data to correspond to correct axis (Page 4 ADXL, quick start)
	//check if data is pos/neg, sign extend accordingly
	if ((adxlData1 & 0xF000) == 0xF000){
		x_Axis_data = (adxlData1 & 0xFF00) | (adxlData0 & 0xFF) | 0xFFFFF000;
	}
	else{
		x_Axis_data = (adxlData1 & 0xFF00) | (adxlData0 & 0xFF);
	}
	
	if ((adxlData2 & 0xF000) == 0xF000){
		y_Axis_data = (adxlData2 & 0xFF00) | (adxlData1 & 0xFF) | 0xFFFFF000;
	}
	else{
		y_Axis_data = (adxlData2 & 0xFF00) | (adxlData1 & 0xFF);
	}
	
	if ((adxlData3 & 0xF000) == 0xF000){
		z_Axis_data = (adxlData3 & 0xFF00) | (adxlData2 & 0xFF) | 0xFFFFF000;
	}
	else{
		z_Axis_data = (adxlData3 & 0xFF00) | (adxlData2 & 0xFF);
	}
		
	return 1;
}

void verify_adxl_spi0(void){	
	
	uint32_t deviceID = 0;
	
	//check SPI connection is solid by doing test read
	//address = 0x00, data = don't care
	
	deviceID = read_adxl(0x00, 0x00);
	
	//device ID should read 0xE5
	//if it does, turn on LED
	if(deviceID == 0xE5){
		
		//set LED pin PA6 as low (LED is active low)
		pioA_ptr->PIO_CODR = PIO_PA6;
	}
}

uint32_t voltage_to_adc(float voltage){
	
	//assuming 3.3v is max. value
	//0v is min. value for adc
	//takes voltage value &
	//returns scaled bit value
	return ((uint32_t) (voltage * 4095)/3.3);
}