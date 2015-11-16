//////////////////////////////////////////////////////////
//              Wristband Project Firmware              //
//              Written by Carlos Hernandez             //
//              San Jose State University               //
//////////////////////////////////////////////////////////

#include <asf.h>

//Global structs for accessing registers
Pio *pioA_ptr = PIOA;
Pio *pioB_ptr = PIOB;
Tc *tc_ptr = TC0;
Adc *adc_ptr = ADC;
Twi *twi4_ptr = TWI4;
Spi *spi5_ptr = SPI5;
Flexcom *fc4_ptr = FLEXCOM4;
Flexcom *fc5_ptr = FLEXCOM5;

//Global variables

//EDA Variables
volatile uint16_t eda_voltage = 0;
volatile uint16_t conductance_value = 0;
volatile uint32_t eda_sum = 0;
volatile uint32_t old_eda_mean = 0;
volatile uint32_t eda_terms = 1;
volatile uint16_t eda_mean = 0;

//EMG Variables
volatile uint16_t emg_voltage = 0;
volatile uint32_t emg_sum = 0;
volatile uint32_t old_emg_mean = 0;
volatile uint32_t emg_terms = 1;
volatile uint32_t emg_mean = 0;

//Heart Rate Variables
volatile int rate[10];                    	// array to hold last ten IBI values
volatile unsigned long sampleCounter = 0; 	// used to determine pulse timing
volatile unsigned long lastBeatTime = 0;  	// used to find IBI
volatile int peak = 2048; 		// used to find peak in pulse wave, seeded (1/2)*(2^12)
volatile int trough = 2048; 	// used to find trough in pulse wave, seeded (1/2)*(2^12)
volatile int thresh = 2100; 	// used to find instant moment of heart beat, seeded (525/1024)*(2^12)
volatile int amp = 400;			// used to hold amplitude of pulse waveform, seeded (100/1024)*(2^12)
volatile int firstBeat = 1;		// used to seed rate array so we startup with reasonable BPM
volatile int secondBeat = 0;	// used to seed rate array so we startup with reasonable BPM
volatile uint8_t BPM;			// int that holds raw Analog in 0. updated every 2mS
volatile int ppg_signal;		// holds the incoming raw data
volatile int IBI = 600;			// int that holds the time interval between beats! Must be seeded!
volatile int Pulse = 0;			// "1" when User's live heartbeat detected. "0" when not a "live beat"
volatile int rate_counter = 0;	// array element counter

//ble variables
volatile uint8_t ble_data[8] = {11, 22, 33, 44, 55, 66, 77, 88};
volatile uint8_t sensor_data[8] = {0};

//Init. variables
volatile uint32_t spi_init_timer = 0;	
volatile uint32_t spi_init_complete = 0;
volatile uint32_t spi_init_begin = 0;
volatile int ble_ready = 0;

//Accelerometer/Fall Detection Variables
volatile int adxl_data_ready = 0;
volatile int32_t x_Axis_data = 0;
volatile int32_t y_Axis_data = 0;
volatile int32_t z_Axis_data = 0;
volatile uint32_t adxl_irq_source = 0;
volatile uint32_t fall_state = 0;
volatile uint32_t fall_counter = 0;
volatile uint8_t fall_alert = 0;

//Buzzer variables
volatile int speaker_rhythm = 0;
volatile int speaker_state = 0;
volatile int data_counter = 0;
volatile int clear_tc = 0;
volatile int x = 0;

//function prototypes
//Initialization Functions
void disable_wdt(void);
void enable_pio_clk(void);
void enable_tc0_channel0_clk(void);
void enable_tc0_channel1_clk(void);
void enable_adc_clk(void);
void enable_spi0_clk(void);
void enable_spi5_clk(void);
void enable_twi4_clk(void);
void init_pio(void);
void init_tc0_channel0(void);
void init_tc0_channel1(void);
void init_adc(void);
void init_ble_twi4(void);
void init_adxl_spi0(void);
void init_adxl_spi5(void);
void init_adxl(void);
void verify_adxl(void);

//Communication Functions (write, read, etc)
uint32_t read_ble(void);
void write_ble(uint32_t data);
void write_mult_ble(volatile uint8_t data[], uint8_t bytes);
void update_ble_data(void);
void write_adxl(uint32_t address, uint32_t data);
int32_t read_adxl(uint32_t address, uint32_t data);
int32_t read_adxl_spi5(uint32_t address, uint32_t data);
int32_t update_accel_data(void);

//Calculation Functions
void fall_detector(void); //runs fall detection algorithm
float skin_conductance(uint32_t adc_reading); //returns skin conductance
uint16_t eda_average(void); //updates eda average value
uint16_t emg_average(void); //updates ekg average value
uint8_t bpm(void); //returns heart rate in beats/minute
uint32_t voltage_to_adc(float voltage); //returns bit value from voltage

//main sensor timer interrupt (sample rate = 500Hz)
void TC0_Handler(void){

// 	//toggle PA29 pin to check ISR timing
//	pioA_ptr->PIO_SODR |= PIO_CODR_P29;
	
		//ADXL initialization timer
	if(spi_init_timer != 1000){
		spi_init_timer++;
	}
	else {
		spi_init_begin = 1;
//		ble_ready = 1;
	}

	//get adc channels 0 & 1 current reading
	adc_ptr->ADC_CR = ADC_CR_START;
	eda_voltage = adc_ptr->ADC_CDR[1];
//	adc_ptr->ADC_CR = ADC_CR_START;
 	emg_voltage = adc_ptr->ADC_CDR[3];
//	adc_ptr->ADC_CR = ADC_CR_START;
//	ecg_voltage = adc_ptr->ADC_CDR[0];
	conductance_value = (uint16_t) skin_conductance(eda_average());
	emg_average();
	
// 	//toggle PA29 pin to check ISR timing
// 	pioA_ptr->PIO_SODR |= PIO_CODR_P29;	
	//get accel. data
  	if (spi_init_complete == 1){
		if(adxl_data_ready == 1){
 			update_accel_data();
		}
		
		//assume interrupt occurred, check source
		adxl_irq_source = read_adxl(0x30, 0x00);
  	}
	 //run fall detection algorithm
	fall_detector();
//	pioA_ptr->PIO_CODR |= PIO_SODR_P29;		
	
	//update bluetooth data from phone (via arduino)
	//send data to phone, if any available
  	if(ble_ready == 1){
//  //		ble_data = read_ble();
// //		update_ble_data();
		sensor_data[0] = x_Axis_data;
		sensor_data[1] = y_Axis_data;
		sensor_data[2] = z_Axis_data;
		sensor_data[3] = 'r';
		sensor_data[4] = 't';
		sensor_data[5] = '!';
		sensor_data[6] = '!';
		sensor_data[7] = '!';
// 		sensor_data[x] = x * 4;
// 		x++;
// 		if(x == 8){
// 			x = 0;
// 		}
		//toggle PA29 pin to check ISR timing
		pioA_ptr->PIO_SODR |= PIO_CODR_P29;		
		write_mult_ble(sensor_data, 8);
//		ble_data[0] = read_ble();
//		update_ble_data();
		//toggle PA29 pin to check timing
		pioA_ptr->PIO_CODR |= PIO_SODR_P29;
// 		for(data_counter = 0; data_counter < 7; data_counter++){
// 			ble_data[data_counter] = read_ble();
 		}	  
// 	}	
	

// 	if(ble_data[0] == 'a'){
// 		//set LED pin PA6 as low (LED is active low)
// 		pioA_ptr->PIO_CODR |= PIO_PA6;
// 		speaker_state = 0;	
// 	}
// 	else if(ble_data[0] == 'b'){
// 		
// 		speaker_state = 3;
// 	}
	
// 	if(ble_data == 3){
// 		//set LED pin PA6 as low (LED is active low)
// 		pioA_ptr->PIO_CODR |= PIO_PA6;
// 		speaker_state = 0;
// 	}
// // 	else if(ble_data == 4){
// // 		//set LED pin PA6 as high (LED is active low)
// // 		pioA_ptr->PIO_SODR |= PIO_PA6;
// // 		speaker_state = 3;
// // 	}

// 	//Beep speaker on 0.25 seconds, off 0.25 seconds
// 	if(speaker_rhythm != 125 && speaker_state != 3){
// 		speaker_rhythm++;
// 	}
// 	else{
// 		if(speaker_state == 0){
// 			
// 			//Enable TIOA1 (max volume) aka enable tc clock & start tc
// 			tc_ptr->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
// 			speaker_state = 1;
// 		}
// 		else if(speaker_state == 1){
// 			
// 			//disable speaker
// 			tc_ptr->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKDIS;
// 			speaker_state = 0;
// 		}
// 		speaker_rhythm = 0;
// 	}
	
// 	//if ADC reads more than threshold, turn on buzzer
// 	if( (eda_voltage > voltage_to_adc(2.1)) 
// 		|| (emg_voltage > voltage_to_adc(0.101)) ){
// 		
// 		// Enable TIOA1 (max volume) aka enable tc clock & start tc
// 		tc_ptr->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
// 		fall_alert = 1;
// 		//send alert via bluetooth if threshold met
// 		if(ble_ready == 1){			
// 	//		write_ble(3);	
// 		}	
// 	}	
// 	else{
// 		//turn off buzzer
// 		tc_ptr->TC_CHANNEL[1].TC_CCR_CLKDIS;
// 		fall_alert = 0; 
// 	}
// 	
	//clear TC0 interrupt flags
	tc_ptr->TC_CHANNEL[0].TC_SR;
	
	//toggle PA29 pin to check timing
//	pioA_ptr->PIO_CODR |= PIO_SODR_P29;
}

void PIOA_Handler(void){

	//toggle PA29 pin to check ISR timing
/*	pioA_ptr->PIO_CODR |= PIO_CODR_P29;*/
	
	//check interrupt source on PA24 pins
	if(pioA_ptr->PIO_ISR & PIO_ISR_P24){
		adxl_data_ready = 1;
// 		//set LED pin PA6 as low (LED is active low)
// 		pioA_ptr->PIO_CODR |= PIO_PA6;
	}
	//clear ISR flag
	pioA_ptr->PIO_ISR;
	
	//toggle PA29 pin to check ISR timing
/*	pioA_ptr->PIO_SODR |= PIO_SODR_P29;	*/
}

//main function
int main (void)
{
	//Init. system clock (PLLA aka 120 Mhz)
	//Uses EXTERNAL crystal oscillator
	//can be switched by redefining "CONFIG_SYSCLK_SOURCE"
	//see conf_clock.h for possible clock sources

	sysclk_init();
	disable_wdt();
	init_pio();	
	init_tc0_channel0();
	init_tc0_channel1();
	init_adc();
	init_adxl_spi5();
	init_ble_twi4();
	init_adxl();
	
	//empty while loop to run SAMG55 indefinitely
	while (1) {}
}

void disable_wdt(void){
	
	//disable watchdog timer
	WDT->WDT_MR = WDT_MR_WDDIS;
}

void enable_pio_clk(void){

	//Enable PIOA clock (ID: 11)
	PMC->PMC_PCER0 |= PMC_PCER0_PID11;

	//Enable PIOB clock (ID: 12)
	PMC->PMC_PCER0 |= PMC_PCER0_PID12;	
}

void enable_tc0_channel0_clk(void){

	//enable TC0 clock (ID: 23) in PMC
	PMC->PMC_PCER0 |= PMC_PCER0_PID23;
}

void enable_tc0_channel1_clk(void){
	
	//enable TC clock (ID: 24) in PMC
	PMC->PMC_PCER0 |= PMC_PCER0_PID24;
}

void enable_adc_clk(void){
	
	//enable ADC clock (ID: 29)
	PMC->PMC_PCER0 |= PMC_PCER0_PID29;
}

void enable_twi4_clk(void){
	
	//enable TWI4 clock (ID:20)
	//use PCK7 (already set from enabling SPI5 clk)
	PMC->PMC_PCER0 |= PMC_PCER0_PID20;
}

void enable_spi5_clk(void){
	
	/* SPI will use main clock as source
	b/c it is limited to 5 Mhz maximum */
	
	//enable SPI5 (a.k.a. FLEXCOM5) clk (ID: 21)
	PMC->PMC_PCER0 |= PMC_PCER0_PID21;	
	
	//disable PCK6 (for FLEXCOM0 & FLEXCOM1 , 2, 3) to configure
	PMC->PMC_SCDR = PMC_SCDR_PCK7;
	
	//set main clock as source for PCK7 (8Mhz) scale to 8Mhz/2
	PMC->PMC_PCK[PMC_PCK_7] = PMC_PCK_CSS_MAIN_CLK | PMC_PCK_PRES_CLK_2;
	
	//enable PCK7
	PMC->PMC_SCER = PMC_SCER_PCK7;
	
	//wait for PCK7 to be ready
	while(!(PMC->PMC_SR & PMC_SR_PCKRDY7)){}
}

void init_pio(void){

	enable_pio_clk();
	
	//enable pin PA6 as output (LED)
	pioA_ptr->PIO_OER |= PIO_PA6;
	
	//enable pin PA29 as output (Debugging)
	pioA_ptr->PIO_OER |= PIO_PA29;
	
	//enable pin PB11 as output (debugging)
//	pioB_ptr->PIO_OER |= PIO_PB11;

	//set LED pin PA6 as high (LED is active low)
	pioA_ptr->PIO_SODR |= PIO_PA6;
	
	//enable switch (pull up resistor) [PA2]
	pioA_ptr->PIO_PUER |= PIO_PA2;
	
	//enable switch pin control by PIO [PA2]
	pioA_ptr->PIO_PER |= PIO_PA2;
	
	//code to check if button is pressed
	//if(pioA_ptr->PIO_PDSR & PIO_PA2){}	
}

void init_tc0_channel0(void){

//uncomment this section to verify TC0 running
//otherwise do not enable (to save power)		
	//set peripheral function for tc (B function) on pin PA0
	pioA_ptr->PIO_ABCDSR[0] |= PIO_ABCDSR_P0;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P0;
	
// 	//set peripheral function for tc (B function) on pin PA1
// 	pioA_ptr->PIO_ABCDSR[0] |= PIO_ABCDSR_P1;
// 	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P1;
	
	//disable PIO control of PA0 & PA1 so TC can control pins
	pioA_ptr->PIO_PDR |= PIO_PA0;
//	pioA_ptr->PIO_PDR |= PIO_PA1;
	
	//enable pmc periph clock for tc
	enable_tc0_channel0_clk();
	
	/*TC0 Setup*/
	//temporarily disable TC clk input
	tc_ptr->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
		
	//set wave mode and to reset on RC match
	//also set muxed pin to toggle on RA & RC match (scope debugging)
	tc_ptr->TC_CHANNEL[0].TC_CMR = 
		TC_CMR_WAVE						//set TC for wave mode
		| TC_CMR_WAVSEL_UP_RC			//count up to RC value
		| TC_CMR_TCCLKS_TIMER_CLOCK2	// = 120Mhz (Master Clock) * (1/2)
		| TC_CMR_EEVT_XC0				//set xc0 so TIOB is not used as input
		| TC_CMR_ACPA_SET				//set PA0 on RA match
		| TC_CMR_ACPC_CLEAR				//clear PA0 on RC match
		| TC_CMR_BCPB_SET				//set PA1 on RB match
		| TC_CMR_BCPC_CLEAR;			//clear PA1 on RC match
	
	//set period & duty cycle (RC value = (120Mhz * prescaler)/(goal frequency))
// 	tc_ptr->TC_CHANNEL[0].TC_RA = 0x7530; //duty cycle for TIOA
// 	tc_ptr->TC_CHANNEL[0].TC_RB = 0x0000; //duty cycle for TIOB	
// 	tc_ptr->TC_CHANNEL[0].TC_RC = 0xEA60; //period (for TIOA & TIOB)
	tc_ptr->TC_CHANNEL[0].TC_RA = 0x3A98;
	tc_ptr->TC_CHANNEL[0].TC_RC = 0x7530;
	
	//enable interrupt on RC compare match
	tc_ptr->TC_CHANNEL[0].TC_IER = TC_IER_CPAS;
	
	//enable tc clock & start tc
	tc_ptr->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;	
	/*End TC0 Setup*/	
	
// 	//Enable TC0 Interrupt in NVIC
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn, 0);
	NVIC_EnableIRQ(TC0_IRQn);
}

void init_tc0_channel1(void){
	
	//set peripheral function for tc (B function) on pin PA23
	pioA_ptr->PIO_ABCDSR[0] |= PIO_ABCDSR_P23;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P23;
	
	//disable PIO control of PA23 so TC can control pins
	pioA_ptr->PIO_PDR |= PIO_PA23;

	//enable pmc periph clock for tc
	enable_tc0_channel1_clk();
	
	/*TC0 Channel 1 Setup*/
	//temporarily disable TC clk input
	tc_ptr->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKDIS;
	
	//set wave mode and to reset on RC match
	//also set muxed pin to toggle on RA & RC match (scope debugging)
	tc_ptr->TC_CHANNEL[1].TC_CMR =
	TC_CMR_WAVE						//set TC for wave mode
	| TC_CMR_WAVSEL_UP_RC			//count up to RC value
	| TC_CMR_TCCLKS_TIMER_CLOCK1	// = 120Mhz (Master Clock) * (1/2)
	| TC_CMR_ACPA_SET				//set PA23 on RA match
	| TC_CMR_ACPC_CLEAR;				//clear PA23 on RC match
	
	//set period & duty cycle (RC value = (120Mhz * prescaler)/(goal frequency))
	tc_ptr->TC_CHANNEL[1].TC_RA = 0x1D4C; //duty cycle for TIOA
	tc_ptr->TC_CHANNEL[1].TC_RB = 0x1D4C; //duty cycle for TIOB
	tc_ptr->TC_CHANNEL[1].TC_RC = 0x3A98; //period (for TIOA & TIOB)
	
	//enable tc clock & start tc
//	tc_ptr->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
	/*End TC0 Channel 1Setup*/	
}

void init_adc(void){
	
	//set prescaler for ADC clk (max), track time (max)
	//set MR Transfer to 2 (default)
	adc_ptr->ADC_MR =
		ADC_MR_TRACKTIM(490)
		| ADC_MR_TRANSFER(2)
		| ADC_MR_PRESCAL(255);
		
	//enable ADC channel 0 (PA17 - Heart rate sensor)
	adc_ptr->ADC_CHER = ADC_CHER_CH0;
	
	//enable ADC channel 1 (PA18 - EDA sensor)
	adc_ptr->ADC_CHER = ADC_CHER_CH1;
	
	//enable ADC channel 3 (PA20 - EMG sensor)
	adc_ptr->ADC_CHER = ADC_CHER_CH3;

	enable_adc_clk();	
}

void init_ble_twi4(void){
	
	//set flexcom4 mode to TWI
	fc4_ptr->FLEXCOM_MR = FLEXCOM_MR_OPMODE_TWI;
	
	//set peripheral function A for TWI on PB10 (Data)
	pioB_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P10;
	pioB_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P10;

	//set peripheral function A for SPI on pin PB11 (Clk)
	pioB_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P11;
	pioB_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P11;
	
	//disable PIO control of PB10, PB11 so TWI4 can control pins
	pioB_ptr->PIO_PDR |= PIO_PB10;
	pioB_ptr->PIO_PDR |= PIO_PB11;
	
	//set device address: 0x08
	twi4_ptr->TWI_MMR = TWI_MMR_DADR(0x08);
	
	//clock waveform generator (set to ~139kHz data rate)
	twi4_ptr->TWI_CWGR = TWI_CWGR_BRSRCCLK_PMC_PCK
	| TWI_CWGR_CHDIV(0x03)
	| TWI_CWGR_CLDIV(0x03)
	| TWI_CWGR_CKDIV(0x02);
	
	//disable slave mode, disable high speed mode
	twi4_ptr->TWI_CR = TWI_CR_SVDIS | TWI_CR_HSDIS;
	
	enable_twi4_clk();
	
	//enable master mode
	twi4_ptr->TWI_CR |= TWI_CR_MSEN;
	
	ble_ready = 1;
}

void write_ble(uint32_t data){
	
	//set master to write mode (MREAD = 0)
	twi4_ptr->TWI_MMR &= ~TWI_MMR_MREAD;
	
	//write: (automatically handles start condition)
	twi4_ptr->TWI_THR = data;
	
	//send stop command
	twi4_ptr->TWI_CR = TWI_CR_STOP;
	
	//wait for acknowledge from slave
	while(!(twi4_ptr->TWI_SR & TWI_SR_TXRDY)){}
	
	//wait for stop acknowledge from slave
	while(!(twi4_ptr->TWI_SR & TWI_SR_TXCOMP)){}
}

void write_mult_ble(volatile uint8_t data[], uint8_t bytes){
	
	//set master to write mode (MREAD = 0)
	twi4_ptr->TWI_MMR &= ~TWI_MMR_MREAD;
	
	uint8_t element = 0;
	
	//keep transferring data as long as 
	for(element = 0; element < bytes; element++){
		
		//write: (automatically handles start condition)
		twi4_ptr->TWI_THR = data[element];

		//wait for acknowledge from slave
		while(!(twi4_ptr->TWI_SR & TWI_SR_TXRDY)){}
	}
	
	//send stop command
	twi4_ptr->TWI_CR = TWI_CR_STOP;
	
	//wait for stop acknowledge from slave
	while(!(twi4_ptr->TWI_SR & TWI_SR_TXCOMP)){}	
}

uint32_t read_ble(void){
	
	uint32_t read_data = 0;
	
	//set master to read mode (MREAD = 1)
	twi4_ptr->TWI_MMR |= TWI_MMR_MREAD;
	
	//send start and stop conditions
	twi4_ptr->TWI_CR = TWI_CR_START | TWI_CR_STOP;
	
	//wait for received data to be ready	
	while(!(twi4_ptr->TWI_SR & TWI_SR_RXRDY)){}
		
	read_data = twi4_ptr->TWI_RHR;
			
	//wait for transfer to complete
	while(!(twi4_ptr->TWI_SR & TWI_SR_TXCOMP)){}
			
	return read_data;
}

void update_ble_data(void){
	
	//element counter
	uint32_t ble_byte = 0;
	
	//set master to read mode (MREAD = 1)
	twi4_ptr->TWI_MMR |= TWI_MMR_MREAD;
	
	for(ble_byte = 0; ble_byte < 7; ble_byte++){
		
		//send start conditions
		twi4_ptr->TWI_CR = TWI_CR_START;
	
		//wait for received data to be ready
		while(!(twi4_ptr->TWI_SR & TWI_SR_RXRDY)){}
	
		ble_data[ble_byte] = twi4_ptr->TWI_RHR;
	}
	
	twi4_ptr->TWI_CR = TWI_CR_STOP;
	
	//wait for received data to be ready
	while(!(twi4_ptr->TWI_SR & TWI_SR_RXRDY)){}
		
	ble_data[ble_byte] = twi4_ptr->TWI_RHR;

	//wait for transfer to complete
	while(!(twi4_ptr->TWI_SR & TWI_SR_TXCOMP)){}	
}

void init_adxl_spi5(void){
	
	//disable SPI to configure
	spi5_ptr->SPI_CR = SPI_CR_SPIDIS;
	
	//reset SPI
	spi5_ptr->SPI_CR = SPI_CR_SWRST;
	
	//set peripheral function A for SPI on pin PA11 (NPCS0/NSS/CS)
	pioA_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P11;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P11;	
	
	//set peripheral function A for SPI on pin PA12 (MISO/SDO)
	pioA_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P12;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P12;

	//set peripheral function A for SPI on pin PA13 (MOSI/SDA)
	pioA_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P13;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P13;
	
	//set peripheral function A for SPI on pin PA14 (SPCK/SCL)
	pioA_ptr->PIO_ABCDSR[0] &= ~PIO_ABCDSR_P14;
	pioA_ptr->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P14;
	
	//enable PIO control of PA24 for use as input for INT1
	pioA_ptr->PIO_PER |= PIO_PA24;

	//disable output on PA24
	pioA_ptr->PIO_ODR |= PIO_PA24;

	//enable interrupt on PA24
	pioA_ptr->PIO_IER |= PIO_PA24;

	//enable additional interrupt settings for PA24
	pioA_ptr->PIO_AIMER |= PIO_PA24;

	//enable edge detection for PA26
	pioA_ptr->PIO_ESR |= PIO_PA24;

	//set to detect rising edge for PA24 (INT1)
	pioA_ptr->PIO_REHLSR |= PIO_PA24;
	
	//disable PIO control of PA11, PA12, PA13, PA14
	//so spi5 can control pins
	pioA_ptr->PIO_PDR |= PIO_PA11;
	pioA_ptr->PIO_PDR |= PIO_PA12;
	pioA_ptr->PIO_PDR |= PIO_PA13;
	pioA_ptr->PIO_PDR |= PIO_PA14;
	
	//set flexcom5 mode to SPI
	fc5_ptr->FLEXCOM_MR = FLEXCOM_MR_OPMODE_SPI;
	
	//set as master, use peripheral clock, mode fault disable
	spi5_ptr->SPI_MR = SPI_MR_MSTR | SPI_MR_BRSRCCLK_PMC_PCK
	| SPI_MR_PCS(0) | SPI_MR_MODFDIS;
	
	//set to fixed peripheral mode
	spi5_ptr->SPI_MR &= ~SPI_MR_PS;

	//set to direct connection to peripheral
	spi5_ptr->SPI_MR &= ~SPI_MR_PCSDEC;

	//local loopback disabled
	spi5_ptr->SPI_MR &= ~SPI_MR_LLB;
	
	//chip select settings
	spi5_ptr->SPI_CSR[0] = SPI_CSR_CPOL			//CPOL = 1
	| SPI_CSR_BITS_16_BIT	//16-bit transfers
	| SPI_CSR_SCBR(2)		//bit rate 1/2 pclk
	| SPI_CSR_DLYBS(4)		//delay after cs before sck
	| SPI_CSR_DLYBCT(0);	//0 delay btwn multibyte transfers
	
	//CPHA = 1 (NCPHA = 0)
	spi5_ptr->SPI_CSR[0] &= ~SPI_CSR_NCPHA;
	
	//clear CSAAT (programmable clock source)
	spi5_ptr->SPI_CSR[0] &= ~SPI_CSR_CSAAT;
	
	//enable PIOA interrupt for freefall (INT1) in NVIC
	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, 1);
	NVIC_EnableIRQ(PIOA_IRQn);

	//next transfer is last
	spi5_ptr->SPI_CR = SPI_CR_LASTXFER;
	
	//wait for adxl to power up
	while(spi_init_begin == 0){}

	enable_spi5_clk();
	
	//enable SPI
	spi5_ptr->SPI_CR = SPI_CR_SPIEN;
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
//	write_adxl(0x25, 0x20);
	
	//set inactivity threshold (0.1875g)
//	write_adxl(0x26, 0x03);
	
	//enable xyz axis for activity & inactivity detection
	//set activity to dc-coupled, inactivity to ac-coupled
//	write_adxl(0x27, 0x7F);
	
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
	spi5_ptr->SPI_TDR = SPI_TDR_TD(data);
		
	//wait for transaction to end
	while((spi5_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
		
	//read data register to clear flag
	data = spi5_ptr->SPI_RDR;
}

//reads single register from ADXL, returns 8 bit value read
int32_t read_adxl(uint32_t address, uint32_t data){
	
	//combine data and address, set read bit
	data |= (address << 8) | 0x8000;
		
	//write to transfer data register
	spi5_ptr->SPI_TDR = SPI_TDR_TD(data);
	
	//wait for transaction to end
	while((spi5_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
	
	//discard don't cares, return 8 bits
	return (spi5_ptr->SPI_RDR & 0xFF);
}

int32_t update_accel_data(void){
	
	//temp variables to save unordered data
	uint32_t adxlData0 = 0;
	uint32_t adxlData1 = 0;
	uint32_t adxlData2 = 0;
	uint32_t adxlData3 = 0;
	
	//write to start data transfer
	//read command (R = 1) multibyte bit on (MB = 1), address 0x32
	spi5_ptr->SPI_TDR = SPI_TDR_TD(0xF200);
	
	//wait for data to load into shift register
	while(!(spi5_ptr->SPI_SR & SPI_SR_TDRE)){}
	
	//start loading next data
	spi5_ptr->SPI_TDR = SPI_TDR_TD(0xFFFF);
	
	//check if read is ready
	while((spi5_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
	
	//save data
	adxlData0 = spi5_ptr->SPI_RDR;
	
	//clear ready flag
	adxl_data_ready = 0;
	
	//wait for data to load into shift register
	while(!(spi5_ptr->SPI_SR & SPI_SR_TDRE)){}

	//start loading next data
	spi5_ptr->SPI_TDR = SPI_TDR_TD(0xFFFF);
	
	//check if read is ready
	while((spi5_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
	
	//save data
	adxlData1 = spi5_ptr->SPI_RDR;
	
	//wait for data to load into shift register
	while(!(spi5_ptr->SPI_SR & SPI_SR_TDRE)){}

	//start loading next data
	spi5_ptr->SPI_TDR = SPI_TDR_TD(0xFFFF);
	
	//check if read is ready
	while((spi5_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
	
	//save data
	adxlData2 = spi5_ptr->SPI_RDR;
	
	//check if read is ready
	while((spi5_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
	
	//save data
	adxlData3 = spi5_ptr->SPI_RDR;
	
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
	
	//dummy code to verify function ends	
	return 1;
}

void verify_adxl(void){	
	
	uint32_t deviceID = 0;
	
	//check SPI connection is solid by doing test read
	//address = 0x00, data/command = don't care
	
	deviceID = read_adxl_spi5(0x00, 0x00);
	
	//device ID should read 0xE5
	//if it does, turn on LED
	if(deviceID == 0xE5){
		
		//set LED pin PA6 as low (LED is active low)
		pioA_ptr->PIO_CODR |= PIO_PA6;
	}
}

int32_t read_adxl_spi5(uint32_t address, uint32_t data){
	
	//combine data and address, set read bit
	data |= (address << 8) | 0x8000;
	
	//write to transfer data register
	spi5_ptr->SPI_TDR = SPI_TDR_TD(data);
	
	//wait for transaction to end
	while((spi5_ptr->SPI_SR & SPI_SR_RDRF) == 0){}
	
	//discard don't cares, return 8 bits
	return (spi5_ptr->SPI_RDR & 0xFF);
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

float skin_conductance(uint32_t adc_reading){

//code modified from EDA sensor from E-Health Kit
//originally on Arduino/Raspberry platform
//modified for SAMG55
/*
 *  eHealth sensor platform for Arduino and Raspberry from Cooking-hacks.
 *
 *  Description: "The e-Health Sensor Shield allows Arduino and Raspberry Pi 
 *  users to perform biometric and medical applications by using 9 different 
 *  sensors: Pulse and Oxygen in Blood Sensor (SPO2), Airflow Sensor (Breathing),
 *  Body Temperature, Electrocardiogram Sensor (ECG), Glucometer, Galvanic Skin
 *  Response Sensor (GSR - Sweating), Blood Pressure (Sphygmomanometer) and 
 *  Patient Position (Accelerometer)."
 *
 *  Copyright (C) 2012 Libelium Comunicaciones Distribuidas S.L.
 *  http://www.libelium.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Version 2.0
 *  Author: Luis Martín & Ahmad Saad
 */
	// Local variable declaration.
	float conductance;
		
	//use adc reading to extract conductance value
	conductance = 1/(2000000*((((adc_reading*3.3)/65535) - 0.5) / 100000));
	
	
	//return reasonable values, else error code
	if (conductance > 1.0){
		 	return conductance;
	}
	else{
		return -1.0;	
	}
}

uint8_t bpm(void){
	
	/*The MIT License (MIT)

	Copyright (c) 2015 Pulse Sensor

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.*/
	
	/*Modified by Carlos Hernandez, SJSU for use in SAMG55 MCU*/
	
	ppg_signal = adc_ptr->ADC_CDR[0];		// read the Pulse Sensor
	sampleCounter += 2;						// keep track of the time in mS with this variable
	int N = sampleCounter - lastBeatTime;	// monitor the time since the last beat to avoid noise

	//  find the peak and trough of the pulse wave
	if(ppg_signal < thresh && N > (IBI/5)*3){     	// avoid dichrotic noise by waiting 3/5 of last IBI
		if (ppg_signal < trough){					// trough is the trough
			trough = ppg_signal;					// keep track of lowest point in pulse wave
		}
	}

	if(ppg_signal > thresh && ppg_signal > peak){	// thresh condition helps avoid noise
		peak = ppg_signal;							// keep track of highest point in pulse wave
	}

	//  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
	// ppg_signal surges up in value every time there is a pulse
	if (N > 250){									// avoid high frequency noise
		if ( (ppg_signal > thresh) && (Pulse == 0) && (N > (IBI/5)*3) ){
			Pulse = 1;								// set the Pulse flag when we think there is a pulse
			IBI = sampleCounter - lastBeatTime;		// measure time between beats in mS
			lastBeatTime = sampleCounter;			// keep track of time for next pulse

			if(secondBeat == 1){					// if this is the second beat, secondBeat == 1
				secondBeat = 0;						// clear secondBeat flag									
				for(rate_counter = 0; rate_counter <= 9; rate_counter++){
					rate[rate_counter] = IBI;		// seed running total to get a realistic BPM at startup
				}
			}

			if(firstBeat == 1){  // if it's the first time we found a beat, firstBeat == 1
				firstBeat = 0;                   	// clear firstBeat flag
				secondBeat = 1;						// set the second beat flag
				return 0;							// IBI value is unreliable so discard it
			}

			// keep a running total of the last 10 IBI values
			int runningTotal = 0;                  			// clear the runningTotal variable
			for(rate_counter = 0; rate_counter <= 8; rate_counter++){	// shift data in the rate array
				rate[rate_counter] = rate[rate_counter + 1];			// and drop the oldest IBI value
				runningTotal += rate[rate_counter];              		// add up the 9 oldest IBI values
			}

			rate[9] = IBI;					// add the latest IBI to the rate array
			runningTotal += rate[9];		// add the latest IBI to runningTotal
			runningTotal /= 10;				// average the last 10 IBI values
			BPM = 60000/runningTotal;		// how many beats can fit into a minute? that's BPM!
		}
	}

	if (ppg_signal < thresh && Pulse == 1){	// when the values are going down, the beat is over
		Pulse = 0;							// reset the Pulse flag so we can do it again
		amp = peak - trough;				// get amplitude of the pulse wave
		thresh = amp/2 + trough;			// set thresh at 50% of the amplitude
		peak = thresh;						// reset these for next time
		trough = thresh;
	}

	if (N > 2500){							// if 2.5 seconds go by without a beat
		thresh = 2100;						// set thresh default
		peak = 2048;						// set peak default
		trough = 2048;						// set trough default
		lastBeatTime = sampleCounter;		// bring the lastBeatTime up to date
		firstBeat = 1;						// set these to avoid noise
		secondBeat = 0;						// when we get the heartbeat back
	}
	return BPM;
}

uint16_t eda_average(void){
	
	eda_sum = (old_eda_mean * (eda_terms - 1)) + eda_voltage;
	eda_mean = eda_sum / eda_terms;
	eda_terms++;
	
	//prevent divide by zero
	if(eda_terms == 0){
		eda_terms = 1;
	}
	return eda_mean;
}

uint16_t emg_average(void){
	
	emg_sum = (old_emg_mean * (emg_terms - 1)) + emg_voltage;
	emg_mean = emg_sum / emg_terms;
	emg_terms++;
	
	//prevent divide by zero
	if(emg_terms == 0){
		emg_terms = 1;
	}
	return emg_mean;
}