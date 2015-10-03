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
Twi *twi3_ptr = TWI3;
Flexcom *fc0_ptr = FLEXCOM0;
Flexcom *fc3_ptr = FLEXCOM3;

//Global variables
uint32_t eda_voltage = 0;
uint32_t emg_voltage = 0;
uint32_t ecg_voltage = 0;
uint32_t old_eda_voltage = 0;
uint32_t old_emg_voltage = 0;
uint32_t old_ecg_voltage = 0;
uint32_t spi_init_timer = 0;
uint32_t ble_data = 0;
volatile int spi_init_complete = 0;
volatile int spi_init_begin = 0;
volatile int adxl_data_ready = 0;
volatile int ble_ready = 0;
volatile int32_t x_Axis_data = 0;
volatile int32_t y_Axis_data = 0;
volatile int32_t z_Axis_data = 0;
volatile int32_t adxl_irq_source = 0;
volatile uint32_t fall_state = 0;
volatile uint32_t fall_counter = 0;

//function prototypes
void enable_pio_clk(void);
void enable_tc0_clk(void);
void enable_adc_clk(void);
void enable_twi3_clk(void);
void enable_spi0_clk(void);
void init_pio(void);
void init_tc0(void);
void init_adc(void);
void init_ble_twi3(void);
void init_adxl_spi0(void);
void init_adxl(void);
void verify_adxl_spi0(void);
uint32_t read_ble(void);
void write_ble(uint32_t data);
void write_adxl(uint32_t address, uint32_t data);
int32_t read_adxl(uint32_t address, uint32_t data);
int32_t update_accel_data(void);
uint32_t voltage_to_adc(float voltage);
void fall_detector(void);

//main sensor timer interrupt (sample rate = 1kHz)
void TC0_Handler(void){
	
	//toggle PA29 pin to check ISR timing
	pioA_ptr->PIO_CODR |= PIO_CODR_P29;

		//ADXL initialization timer
	if(spi_init_timer != 100){
		spi_init_timer++;
	}
	else {
		spi_init_begin = 1;
	}
	
	//get adc channels 0 & 1 current reading
	adc_ptr->ADC_CR = ADC_CR_START;
	eda_voltage = adc_ptr->ADC_CDR[0];
// 	emg_voltage = adc_ptr->ADC_CDR[1];
// 	ecg_voltage = adc_ptr->ADC_CDR[2];

	//get accel. data
  	if (spi_init_complete == 1){
		if(adxl_data_ready == 1){
 			update_accel_data();
		}
		adxl_irq_source = read_adxl(0x30, 0x00);
  	}
	
	//update bluetooth data
// 	if(ble_ready == 1){
// 	//	ble_data = read_ble(); 
// 	}
	
	
	//run fall detection algorithm
//	fall_detector();
	
	//if ADC reads more than threshold, turn on buzzer
	if( (eda_voltage > voltage_to_adc(1.3)) 
		|| (emg_voltage > voltage_to_adc(0.101)) ){
		
		// 50% duty cycle on TIOB (max volume)	
		tc0_ptr->TC_CHANNEL[0].TC_RB = 0x7530; 
// 		old_eda_voltage = eda_voltage;
// 		old_emg_voltage = emg_voltage;
// 		old_ecg_voltage = ecg_voltage;

		//send alert via bluetooth if threshold met
		if(ble_ready == 1){
			write_ble('x');	
		}
		
	}	
	else{
		
		//0% duty cycle on TIOB (buzzer off)
		tc0_ptr->TC_CHANNEL[0].TC_RB = 0x0000; 
// 		old_eda_voltage = eda_voltage;
// 		old_emg_voltage = emg_voltage;
// 		old_ecg_voltage = ecg_voltage;
		if(ble_ready == 1){
//			write_ble(2);
		}
	}
	
	//clear TC0 interrupt flags
	tc0_ptr->TC_CHANNEL[0].TC_SR;
	
	//toggle PA29 pin to check timing
	pioA_ptr->PIO_SODR |= PIO_SODR_P29;
}

void PIOA_Handler(void){

	//toggle PA29 pin to check ISR timing
/*	pioA_ptr->PIO_CODR |= PIO_CODR_P29;*/
	
	if(pioA_ptr->PIO_ISR & PIO_ISR_P26){		
		
		//if LED on
		if(pioA_ptr->PIO_ODSR & PIO_PA6){
			
			//turn LED ON
			pioA_ptr->PIO_CODR |= PIO_PA6;
		}
		
		else{
			
			//turn LED OFF
			pioA_ptr->PIO_SODR |= PIO_PA6;
		}
	}
	//clear ISR flag
	pioA_ptr->PIO_ISR;
	
	//toggle PA29 pin to check ISR timing
/*	pioA_ptr->PIO_SODR |= PIO_SODR_P29;	*/
}

void PIOB_Handler(void){

// 	//toggle PA29 pin to check ISR timing
// 	pioA_ptr->PIO_CODR |= PIO_CODR_P29;
	
	//check interrupt source on PB pins
	if(pioB_ptr->PIO_ISR & PIO_ISR_P9){
		adxl_data_ready = 1;
	}
	
	//clear ISR flag
	pioB_ptr->PIO_ISR;
	
// 	//toggle PA29 pin to check ISR timing
// 	pioA_ptr->PIO_SODR |= PIO_SODR_P29;
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
	init_tc0();
	init_adc();
	init_adxl_spi0();
	init_ble_twi3();
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

void enable_tc0_clk(void){

	//enable TC clock (ID: 23) in PMC
	PMC->PMC_PCER0 |= PMC_PCER0_PID23;
}

void enable_adc_clk(void){
	
	//enable ADC clock (ID: 29)
	PMC->PMC_PCER0 |= PMC_PCER0_PID29;
}

void enable_twi3_clk(void){
	
	//enable TWI3 clock (ID:19)
	//use PCK6 (already set from enable SPI0 clk)
	//enable TWI3 aka FLEXCOM3 clk ID: 19
	PMC->PMC_PCER0 |= PMC_PCER0_PID19;
	
}

void enable_spi0_clk(void){
	
	/* SPI will use main clock as source
	b/c it is limited to 5 Mhz maximum */
	
	//enable SPI0 (a.k.a. FLEXCOM0) clk (ID: 8)
	PMC->PMC_PCER0 |= PMC_PCER0_PID8;	
	
	//disable PCK6 (for FLEXCOM0 & FLEXCOM1 , 2, 3) to configure
	PMC->PMC_SCDR = PMC_SCDR_PCK6;
	
	//set main clock as source for PCK6 (8Mhz) scale to 8Mhz/2
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
	
	//enable pin PA29 as output (Debugging)
	pioA_ptr->PIO_OER |= PIO_PA29;

	//set LED pin PA6 as high (LED is active low)
	pioA_ptr->PIO_SODR |= PIO_PA6;
	
	//enable switch (pull up resistor) [PA2]
	pioA_ptr->PIO_PUER |= PIO_PA2;
	
	//enable switch pin control by PIO [PA2]
	pioA_ptr->PIO_PER |= PIO_PA2;
	
	//code to check if button is pressed
	//if(pioA_ptr->PIO_PDSR & PIO_PA2){}	
}

void init_tc0(void){
		
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
	enable_tc0_clk();
	
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
	tc0_ptr->TC_CHANNEL[0].TC_RA = 0x7530; //duty cycle for TIOA
//	tc0_ptr->TC_CHANNEL[0].TC_RB = 0x7530; //duty cycle for TIOB
	tc0_ptr->TC_CHANNEL[0].TC_RB = 0x0000; //duty cycle for TIOB	
	tc0_ptr->TC_CHANNEL[0].TC_RC = 0xEA60; //period (for TIOA & TIOB)
	
	//enable interrupt on RC compare match
	tc0_ptr->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
	
	//enable tc clock & start tc
	tc0_ptr->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;	
	/*End TC0 Setup*/	
	
	//Enable TC0 Interrupt in NVIC
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn, 1);
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

void init_ble_twi3(void){
		
	//set flexcom3 mode to TWI
	fc3_ptr->FLEXCOM_MR = FLEXCOM_MR_OPMODE_TWI;
		
	//set peripheral function A for TWI on PA03 (Data)
	pioA_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P3;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P3;

	//set peripheral function A for SPI on pin PA04 (Clk)
	pioA_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P4;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P4;
	
	//disable PIO control of PA03, PA04 so TWI3 can control pins
	pioA_ptr->PIO_PDR |= PIO_PA3;
	pioA_ptr->PIO_PDR |= PIO_PA4;
	
	//set device address: 0x08
	twi3_ptr->TWI_MMR = TWI_MMR_DADR(0x08);
	
	//clock waveform generator (set to ~69kHz data rate)
	twi3_ptr->TWI_CWGR = TWI_CWGR_BRSRCCLK_PMC_PCK
						 | TWI_CWGR_CHDIV(0x01)
						 | TWI_CWGR_CLDIV(0x01)
						 | TWI_CWGR_CKDIV(0x05);
						 
	//disable slave mode, disable high speed mode
	twi3_ptr->TWI_CR = TWI_CR_SVDIS | TWI_CR_HSDIS;	
	
	enable_twi3_clk();
	
	//enable master mode
	twi3_ptr->TWI_CR |= TWI_CR_MSEN;
	
	ble_ready = 1;
//	write_ble(15);
// 	ble_data = read_ble();
// 	
// 	if(ble_data == 9){
// 		tc0_ptr->TC_CHANNEL[0].TC_RB = 0x7530; //duty cycle for TIOB		
// 	}		 
}

void write_ble(uint32_t data){
	
	//set master to write mode (MREAD = 0)
	twi3_ptr->TWI_MMR &= ~TWI_MMR_MREAD;
	
	//write: (automatically handles start condition)
	twi3_ptr->TWI_THR = data;
	twi3_ptr->TWI_CR = TWI_CR_STOP;
	
	//wait for acknowledge from slave
	while(!(twi3_ptr->TWI_SR & TWI_SR_TXRDY)){}
	
	//wait for stop acknowledge from slave
	while(!(twi3_ptr->TWI_SR & TWI_SR_TXCOMP)){}
}

uint32_t read_ble(void){
	
	uint32_t read_data = 0;
	
	//set master to read mode (MREAD = 1)
	twi3_ptr->TWI_MMR |= TWI_MMR_MREAD;
	
	//send start and stop conditions
	twi3_ptr->TWI_CR = TWI_CR_START | TWI_CR_STOP;
	
	//wait for received data to be ready	
	while(!(twi3_ptr->TWI_SR & TWI_SR_RXRDY)){}
		
	read_data = twi3_ptr->TWI_RHR;
			
	//wait for transfer to complete
	while(!(twi3_ptr->TWI_SR & TWI_SR_TXCOMP)){}
			
	return read_data;
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
	//enable PIO control of PA26 for use as input for INT2
	pioB_ptr->PIO_PER |= PIO_PB9;
	pioA_ptr->PIO_PER |= PIO_PA26;
	
	//disable output on PB9
	//disable output on PA26
	pioB_ptr->PIO_ODR |= PIO_PB9;
	pioA_ptr->PIO_ODR |= PIO_PA26;
	
	//enable interrupt on PB09
	//enable interrupt on PA26
	pioB_ptr->PIO_IER |= PIO_PB9;
	pioA_ptr->PIO_IER |= PIO_PA26;
	
	//enable additional interrupt settings for PB09
	//enable additional interrupt settings for PA26
	pioB_ptr->PIO_AIMER |= PIO_PB9;
	pioA_ptr->PIO_AIMER |= PIO_PA26;
	
	//enable edge detection for PB09
	pioB_ptr->PIO_ESR |= PIO_PB9;
	pioA_ptr->PIO_ESR |= PIO_PA26;
	
	//set to detect rising edge for PB09 (INT1)
	//set to detect rising edge for PA26 (INT2)
	pioB_ptr->PIO_REHLSR |= PIO_PB9;
	pioA_ptr->PIO_REHLSR |= PIO_PA26;
	
	//disable PIO control of PA09, PB00, PA26, & PA10
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
	NVIC_SetPriority(PIOB_IRQn, 0);
	NVIC_EnableIRQ(PIOB_IRQn);

	//enable PIOA interrupt for freefall (INT2) in NVIC
	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, 0);
	NVIC_EnableIRQ(PIOA_IRQn);

	//next transfer is last
	spi0_ptr->SPI_CR = SPI_CR_LASTXFER;
	
	//wait for adxl to power up
	while(spi_init_begin == 0){}

	enable_spi0_clk();	
	
	//enable SPI
	spi0_ptr->SPI_CR = SPI_CR_SPIEN;
}

void init_adxl(void){

	//set to 13-bit mode, +-16g
	write_adxl(0x31, 0x0B);
	
	//set rate to 100 Hz
	write_adxl(0x2C, 0x0F);
	
 	//set freefall min threshold (750 mg)
	write_adxl(0x28, 0x0C);
	
	//set freefall min time (30 ms)
	write_adxl(0x29, 0x06);
	
	//set activity threshold (2g)
	write_adxl(0x25, 0x20);
	
	//set inactivity threshold (0.1875g)
	write_adxl(0x26, 0x03);
	
	//enable xyz axis for activity & inactivity detection
	//set activity to dc-coupled, inactivity to ac-coupled
	write_adxl(0x27, 0x7F);
	
	//set single tap g min treshold (2.5g)
//	write_adxl(0x1D, 0x28);
	
	//set tap duration maximum (50ms);
//	write_adxl(0x21, 0x50);
	
	//enable 3 axis tap detection
//	write_adxl(0x2A, 0x03);
	
	//Start Measurement
	write_adxl(0x2D, 0x08);
	
	//Map data_ready interrupt to INT1 pin
	//others (inc. single tap & freefall) to INT2 pin
	write_adxl(0x2F, 0x7F);
	
	//Enable data_ready, activity, inactivity & freefall interrupts
	write_adxl(0x2E, 0x9C);
	
	//clear interrupts
	read_adxl(0x30, 0x00);
	
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
	//address = 0x00, data/command = don't care
	
	deviceID = read_adxl(0x00, 0x00);
	
	//device ID should read 0xE5
	//if it does, turn on LED
	if(deviceID == 0xE5){
		
		//set LED pin PA6 as low (LED is active low)
		pioA_ptr->PIO_CODR |= PIO_PA6;
	}
}

uint32_t voltage_to_adc(float voltage){
	
	//assuming 3.3v is max. value
	//0v is min. value for adc
	//takes voltage value &
	//returns scaled bit value
	return ((uint32_t) (voltage * 4095)/3.3);
}

void fall_detector(void){
	
	//fall detection state machine
	switch(fall_state){
		
		//default state, no freefall detected
		case 0:
		
			if(adxl_irq_source & 0x30){
				fall_state = 1;
			}
		break;
		
		//freefall detected, start counter
		case 1:
		
			fall_counter++;
		
			//check for impact
			if(adxl_irq_source & 0x10){
				fall_state = 2;
				fall_counter = 0;
			}
		
			if(fall_counter > 200){
				fall_state = 0;
				fall_counter = 0;
			}
		break;
		
		//check	for inactivity after impact
		case 2:
		
			fall_counter++;
		
			//check inactivity trigger
			if(adxl_irq_source & 0x08){
				fall_state = 3;
				fall_counter = 0;
			}
		
			//timeout check
			if(fall_counter > 3500){
				fall_state = 0;
				fall_counter = 0;
			}
		break;
		
		//check if final state different from initial
		case 3:
		
			//compare initial xyz vector with current xyz
			fall_state	= 4;
		break;
		
		//check how long inactivity is
		case 4:
		
			fall_counter++;
			
			//disable interrupts
			write_adxl(0x2E, 0x80);
		
			//set activity threshold to 0.5g
			write_adxl(0x25, 0x08);
		
			//set activity to ac-coupled
			write_adxl(0x27, 0xFF);
		
			//set inactivity threshold to 0.1875g
			write_adxl(0x26, 0x0A);
		
			//enable interrupts
			write_adxl(0x2E, 0x9C);
		
			//check if activity int. triggered
			if(adxl_irq_source & 0x10){
				fall_state = 0;
				fall_counter = 0;
			}
		
			//check if 10s inactivity triggered
			else if (adxl_irq_source & 0x08){
				fall_counter = 0;
				fall_state = 0;
				//generate critical alert
			}
		
			//timeout check
			if(fall_counter > 10500){
				fall_state = 0;
				fall_counter = 0;
			}
		break;
	}
}