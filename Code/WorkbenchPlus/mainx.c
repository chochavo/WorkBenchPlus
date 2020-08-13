#include "main.h"



/* IR Block */
volatile char text[5];
volatile bool nec_ok = false;
volatile unsigned char  i, command, inv_command;
volatile uint8_t nec_state = 0;
volatile unsigned int address;
volatile unsigned long nec_code;
uint8_t IR_state = INIT;
uint16_t time_us = 0;
/* END of IR Block */

volatile bool left_monitor_state = OFF;
volatile bool right_monitor_state = OFF;
volatile bool light_state = OFF;
volatile bool sound_state = OFF;
volatile bool power_state = ON;
volatile uint8_t current_rgb_state = RED;

volatile uint8_t volume_state = 0x00;
volatile uint8_t light_position = 0;

unsigned char ascii[5];



eeprom_write_byte((uint8_t *), );
eeprom_write_block(, (uint8_t *), );

/* Prototypes */
void SPI_write_16bit(uint8_t address_in, uint8_t data_in);
void int_to_ascii(uint16_t time_in);
void enable_interrupt(void);
void disable_interrupt(void);
void enable_timer(void);
void disable_timer(void);
void initInterrupts(void);
void left_monitor(bool statex);
void right_monitor(bool statex);
void Init_light(void);
void Init_UI(void);
void Init_RGB(void);
void Check_Circuits(void);
void light_up(void);
void light_down(void);
void SPI_write_8bit(uint8_t address_in);
void SPI_write_16bit(uint8_t address_in, uint8_t data_in);
uint8_t SPI_read_16bit(uint8_t address_in);
unsigned char SPI_WriteRead(unsigned char dataout);
void Init_SPI(void);
void Check_IR(void);
void disable_all(void);
void enable_all(void);
void set_volume(uint8_t);
void set_LED(uint8_t led_type);
void enable_light(void);
void disable_light(void);
void time_led(uint8_t led_type);
/////////////////////////////////////

void int_to_ascii(uint16_t time_in) {
	for (uint8_t ix = 0; ix < 5; ix++) {
		time_in %= time_in;
		ascii[5 - i] = time_in + 0x30;
	}
}

void enable_interrupt(void) {
PCICR |= (1 << PCIE2);
PCMSK2 |= (1 << PCINT19);
}

void disable_interrupt(void) {
PCICR &= ~(1 << PCIE2);
PCMSK2 &= ~(1 << PCINT19);
}

void enable_timer(void) {
	TCNT2 = 0;
	TCCR2B |= (1 << CS21); // Enable timer (1 us per tick).	
}

void disable_timer(void) {
	TCNT2 = 0;
	TCCR2B &= ~(1 << CS21); // Enable timer (1 us per tick).	
}

ISR(SPI_STC_vect) {	
}

ISR(TIMER1_OVF_vect) {
	nec_state = 0;                                 // Reset decoding process
	TCCR1B = 0;                                    // Disable Timer1 module
}

ISR(PCINT2_vect) {                           // Timer1 interrupt service routine (ISR)
	volatile unsigned int timer_value;
	if(nec_state != 0) {
		timer_value = TCNT1;                         // Store Timer1 value
		TCNT1 = 0;                                   // Reset Timer1
		//writeString(itoa(timer_value,x,10));
	}
switch(nec_state){
	case 0 :                                      // Start receiving IR data (we're at the beginning of 9ms pulse)
	TCNT1  = 0;                                  // Reset Timer1
	TCCR1B = 2;                                  // Enable Timer1 module with 1/8 prescaler ( 2 ticks every 1 us)
	nec_state = 1;                               // Next state: end of 9ms pulse (start of 4.5ms space)
	i = 0;
	return;
	case 1 :                                      // End of 9ms pulse
	if((timer_value > 9500) || (timer_value < 8500)){ 
		        // Invalid interval ==> stop decoding and reset
		nec_state = 0;                             // Reset decoding process
		TCCR1B = 0;                                // Disable Timer1 module
	}
	else
	nec_state = 2;                             // Next state: end of 4.5ms space (start of 562�s pulse)
	return;
	case 2 :                                      // End of 4.5ms space
	if((timer_value > 5000) || (timer_value < 4000)){
		
		nec_state = 0;                             // Reset decoding process
		TCCR1B = 0;                                // Disable Timer1 module
	}
	else
	nec_state = 3;                             // Next state: end of 562�s pulse (start of 562�s or 1687�s space)
	return;
	case 3 :                                      // End of 562�s pulse
	if((timer_value > 700) || (timer_value < 400)){           // Invalid interval ==> stop decoding and reset
		
		TCCR1B = 0;                                // Disable Timer1 module
		nec_state = 0;                             // Reset decoding process
	}
	else
	nec_state = 4;                             // Next state: end of 562�s or 1687�s space
	return;
	case 4 :                                      // End of 562�s or 1687�s space
	if((timer_value > 1800) || (timer_value < 400)){           // Time interval invalid ==> stop decoding
		TCCR1B = 0;                                // Disable Timer1 module
		nec_state = 0;                             // Reset decoding process
		return;
	}
	if( timer_value > 1000)                      // If space width > 1ms (short space)
		nec_code |= (1 << (31 - i));                // Write 1 to bit (31 - i)
		else                                         // If space width < 1ms (long space)
		nec_code &= ~(1 << (31 - i));              // Write 0 to bit (31 - i)
		i++;
		if(i > 31){                                  // If all bits are received
			nec_ok = 1;                                // Decoding process OK
			disable_interrupt();                        // Disable external interrupt (INT0)
			return;
		}
		nec_state = 3;                               // Next state: end of 562�s pulse (start of 562�s or 1687�s space)
	}
}


void initInterrupts(void)
{
	TCCR1A = 0;
	TCCR1B = 0;                                    // Disable Timer1 module
	TCNT1  = 0;                                    // Set Timer1 preload value to 0 (reset)
	TIMSK1 = 1;                                    // enable Timer1 overflow interrupt
	enable_interrupt();
	//sei();
}

void left_monitor(bool statex)
{
	if (statex)
	{
		left_monitor_state = true;
		PORTD |= (1 << MT1_EN);
	}
	else 
	{
		left_monitor_state = false;
		PORTD &= ~(1 << MT1_EN);
	}
}

void right_monitor(bool statex)
{
	if (statex)
	{
		right_monitor_state = true; 
		PORTD |= (1 << MT0_EN);
	}
	else
	{
		right_monitor_state = false;
		PORTD &= ~(1 << MT0_EN);
	}
}

void Init_light(void)
{
	DDRC |= (1 << LIGHT_nINC) | (1 << LIGHT_UnD) | (1 << LIGHT_nCS);
	LIGHT_PORT |= (1 << LIGHT_nINC);
	LIGHT_PORT |= (1 << LIGHT_UnD);
	LIGHT_PORT |= (1 << LIGHT_nCS);
}

void Init_UI(void)
{
	DDRD |= (1 << MT1_EN) | (1 << MT0_EN);
}

void Init_RGB(void)
{
	DDRB |= (1 << LED_R);
	DDRB |= (1 << LED_G);
	DDRD |= (1 << LED_B);	
	set_LED(RED);
	_delay_ms(1000);
	set_LED(GREEN);
	_delay_ms(1000);
	set_LED(BLUE);
	_delay_ms(1000);
}

void Check_Circuits(void)
{
	writeString("\r\nHello\r\n");
	_delay_ms(1000);
	left_monitor(ON);
	_delay_ms(1000);
	right_monitor(ON);
	_delay_ms(1000);
	//LED_out(GREEN | RED | BLUE);
	_delay_ms(1000);
	SPI_write_16bit(WIPER0, 0x00);
	SPI_write_16bit(WIPER1, 0x00);
	_delay_ms(1000);
	PORTB |= (1 << 6);
	_delay_ms(1000);
	PORTB |= (1 << 7);
	_delay_ms(1000);
	PORTD |= (1 << 5);
}

void light_up(void)
{
LIGHT_PORT &= ~(1 << LIGHT_UnD);
LIGHT_PORT &= ~(1 << LIGHT_nINC);
_delay_us(100);
LIGHT_PORT |= (1 << LIGHT_nINC);
_delay_us(100);
}

void light_down(void)
{
	LIGHT_PORT |= (1 << LIGHT_UnD);
	LIGHT_PORT &= ~(1 << LIGHT_nINC);
	_delay_us(100);
	LIGHT_PORT |= (1 << LIGHT_nINC);
	_delay_us(100);
}

void SPI_write_8bit(uint8_t address_in)
{
	SPDR = (address_in << 4) & 0xF0;
	while(!(SPSR & (1<<SPIF)));
	SPI_PORT |= (1<<SPI_CS);
	//_delay_us(1);             // Hold pulse for 1 micro second
	// Disable Latch
	SPI_PORT &= ~(1<<SPI_CS);
}

void SPI_write_16bit(uint8_t address_in, uint8_t data_in)
{
	SPDR = address_in;
	while(!(SPSR & (1<<SPIF)));
	SPDR = data_in;
	while(!(SPSR & (1<<SPIF)));
	SPI_PORT |= (1<<SPI_CS);
	//_delay_us(1);             // Hold pulse for 1 micro second
	// Disable Latch
	SPI_PORT &= ~(1<<SPI_CS);
}

uint8_t SPI_read_16bit(uint8_t address_in)
{
	//SPDR = ((address_in << 4) & 0xF0) || SPI_read_command;
	SPDR = 0b00001100;
	while(!(SPSR & (1<<SPIF)));
	SPDR = 0;
	while(!(SPSR & (1<<SPIF)));
	//SPI_PORT |= (1<<SPI_CS);
	//_delay_us(1);             // Hold pulse for 1 micro second
	// Disable Latch
	//SPI_PORT &= ~(1<<SPI_CS);
	return SPDR;
}

unsigned char SPI_WriteRead(unsigned char dataout)
{
	unsigned char datain;
	// Start transmission (MOSI)
	SPDR = dataout;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	SPDR = 0;
	while(!(SPSR & (1<<SPIF)));
	// Get return Value;
	datain = SPDR;
	// Latch the Output using rising pulse to the RCK Pin
	SPI_PORT |= (1<<SPI_CS);
	//_delay_us(1);             // Hold pulse for 1 micro second
	// Disable Latch
	SPI_PORT &= ~(1<<SPI_CS);
	// Return Serial In Value (MISO)
	return datain;
}

void Init_SPI(void)
{
	// Initial the AVR ATMega168 SPI Peripheral
	// Set MOSI and SCK as output, others as input
	SPI_DDR = (1<<PORTB3)|(1<<PORTB5)|(1<<PORTB2);
	// Latch Disable (RCK Low)
	SPI_PORT &= ~(1<<SPI_CS);
	// Enable SPI, Master, set clock rate fck/2 (maximum)
	SPCR = (1<<SPE)|(1<<MSTR);
	SPSR = (1<<SPI2X);
	_delay_ms(1);
}

bool Check_IR() {
	if(nec_ok) {
		nec_ok = 0;                                  // Reset decoding process
		nec_state = 0;
		TCCR1B = 0;                                  // Disable Timer1 module
		address = nec_code >> 16;
		command = nec_code >> 8;
		inv_command = nec_code;
		writeString("address: ");
		sprintf(text, "%04X", address);
		writeString(text);
		writeString("command: ");
		sprintf(text, "%02X", command);
		writeString(text);
		writeString("n_command: ");
		sprintf(text, "%02X", inv_command);
		writeString(text);
		if (power_state == OFF && (inv_command != KEY_POWER));
		else {
		switch(inv_command) {
			case KEY_VOL_DOWN:
				if (volume_state < 2){
					 volume_state = volume_state;
					 set_LED(RED);
					 _delay_ms(100);
					 set_LED(GREEN);
				}
				else { 
					volume_state -= 2;
					 set_LED(BLUE);
					 _delay_ms(100);
					 set_LED(GREEN);
				}
				SPI_write_16bit(WIPER0, volume_state);
				SPI_write_16bit(WIPER1, volume_state);	
				break;
			
			case KEY_VOL_UP:
				if (volume_state > VOLUME_LIMIT) { 
					volume_state = volume_state;
					 set_LED(RED);
					 _delay_ms(100);
					 set_LED(GREEN);
				}
				else {
					volume_state += 2;
					 set_LED(BLUE);
					 _delay_ms(100);
					 set_LED(GREEN);
				}
				SPI_write_16bit(WIPER0, volume_state);
				SPI_write_16bit(WIPER1, volume_state);
				break;
				
				case KEY_ADJ_R:
				if (volume_state > VOLUME_LIMIT) { 
					volume_state = volume_state;
					 set_LED(RED);
					 _delay_ms(100);
					 set_LED(GREEN);
				}
				else {
					volume_state += 16;
					 set_LED(BLUE);
					 _delay_ms(100);
					 set_LED(GREEN);
				}
				SPI_write_16bit(WIPER0, volume_state);
				SPI_write_16bit(WIPER1, volume_state);
				break;
				
				case KEY_ADJ_L:
				if (volume_state < 16){
					volume_state = volume_state;
					set_LED(RED);
					_delay_ms(100);
					set_LED(GREEN);
				}
				else {
					volume_state -= 16;
					set_LED(BLUE);
					_delay_ms(100);
					set_LED(GREEN);
				}
				SPI_write_16bit(WIPER0, volume_state);
				SPI_write_16bit(WIPER1, volume_state);
				break;
			
			case KEY_CH_UP:
				if (light_position < 5) {
					light_up();
					light_position++;
					light_state = ON;
					 set_LED(BLUE);
					 _delay_ms(100);
					 set_LED(GREEN);
				}
				else //time_led(RED);
				{
					set_LED(RED);
					_delay_ms(100);
					set_LED(GREEN);
				}
				break;
				
			case KEY_CH_DOWN:
				if (light_position > 0) {
					light_state = ON;
					light_down();
					light_position--;
					 set_LED(BLUE);
					 _delay_ms(100);
					 set_LED(GREEN);
				}		
				else {
				set_LED(RED);
				_delay_ms(100);
				set_LED(GREEN);
				}
				break;	
			
			case KEY_A:
				if (left_monitor_state == ON) {
					 left_monitor(OFF);
				}
				else {
					left_monitor(ON);
				}
				set_LED(BLUE);
				_delay_ms(100);
				set_LED(GREEN);
				break;
			
			case KEY_B:
				if (right_monitor_state == ON) right_monitor(OFF);
				else {
					right_monitor(ON);
				}
				set_LED(BLUE);
				_delay_ms(100);
				set_LED(GREEN);
				break;
			
			case KEY_C:
				if (light_state == ON) disable_light();
				else enable_light();
				set_LED(BLUE);
				_delay_ms(100);
				set_LED(GREEN);
				break;
				
			case KEY_POWER:
				if (power_state == ON) disable_all();
				else enable_all();
				break;
			
			case KEY_MUTE:
				if (sound_state == ON) {
					set_volume(0);
					sound_state = OFF;
				}
				else {
					set_volume(volume_state);
					sound_state = ON;
				}
				set_LED(BLUE);
				_delay_ms(100);
				set_LED(GREEN);
				break;
			
			default: break;							
		}
		}
		enable_interrupt();
		return true;
	}
	return false;
}

void disable_all() {
	power_state = OFF;
	left_monitor(OFF);
	right_monitor(OFF);
	disable_light();
	set_volume(0);
	set_LED(RED);
}

void enable_all() {
	power_state = ON;
	left_monitor(ON);;
	right_monitor(ON);
	enable_light();
	set_volume(volume_state);
	set_LED(GREEN);
}

void set_volume(uint8_t volume_in) {
	SPI_write_16bit(WIPER0, volume_in);
	SPI_write_16bit(WIPER1, volume_in);	
}

void Init_light_state() {
	LIGHT_PORT &= ~(1 << LIGHT_nCS);
	_delay_ms(10);
	for (uint8_t i = 0; i < 255; i++) light_down();
	for (uint8_t i = 0; i < 23; i++) light_up();
	light_state = OFF;
	light_position = 0;
}

void disable_light() {
	LIGHT_PORT &= ~(1 << LIGHT_nCS);
	for (uint8_t i = 0; i < light_position; i++) light_down();
	light_state = OFF;
	light_position = 0;
}

void enable_light() {
	LIGHT_PORT &= ~(1 << LIGHT_nCS);
	_delay_ms(10);
	for (uint8_t i = light_position; i < 5; i++) light_up();
	light_position = 5;
	light_state = ON;
}

void set_LED(uint8_t led_type) {
	switch(led_type) {
		case RED: 
			DDRB |= (1 << LED_R);
			PORTB |= (1 << LED_R);
			PORTB &= ~(1 << LED_G);
			PORTD &= ~(1 << LED_B);
			current_rgb_state = RED;
			break;
		case GREEN:
			PORTB &= ~(1 << LED_R);
			DDRB |= (1 << LED_G);
			PORTB |= (1 << LED_G);
			PORTD &= ~(1 << LED_B);
			current_rgb_state = GREEN;
			break;
		case BLUE:
			PORTB &= ~(1 << LED_R);
			PORTB &= ~(1 << LED_G);
			DDRD |= (1 << LED_B);
			PORTD |= (1 << LED_B);
			current_rgb_state = BLUE;
			break;
		default: break;
	}
}

void time_led(uint8_t led_type)
{
	uint8_t rgb_state_imm = current_rgb_state;
	set_LED(led_type);
	_delay_ms(100);
	set_LED(rgb_state_imm);
}

void Init_switch() {
}

void Init_Device() {
	Init_switch();
	Init_RGB();
	set_LED(GREEN);
	Init_light();
	initInterrupts();
	initUART();
	Init_UI();
	Init_SPI();
	Init_LCD();
	Init_LCD_4bit();
	Init_light_state();
	sei();
}

void print_time_string(int second_in, int minute_in, int hour_in) {
	int_to_ascii(hour_in);
	Print_Character(buf[0]);
	Print_Character(buf[1]);
	Print_Character(':');
	int_to_ascii(minute_in);
	Print_Character(buf[0]);
	Print_Character(buf[1]);
	Print_Character(':');
	int_to_ascii(second_in);
	Print_Character(buf[0]);
	Print_Character(buf[1]);
}

void month_string_determine (uint8_t month_in)
{
	switch(month_in)
	{
		case 1: print_string("January"); break;
		case 2: print_string("February"); break;
		case 3: print_string("March"); break;
		case 4: print_string("APR"); break;
		case 5: print_string("MAY"); break;
		case 6: print_string("JUN"); break;
		case 7: print_string("JUL"); break;
		case 8: print_string("AUG"); break;
		case 9: print_string("SEP"); break;
		case 10:print_string("OCT"); break;
		case 11:print_string("NOV"); break;
		case 12:print_string("DEC"); break;
		default: break;
	}
}

void print_date_string(int day_in, int month_in)
{
	Print_Character('|');
	int_to_ascii(day_in);
	Print_Character(buf[0]);
	Print_Character(buf[1]);
	Print_Character('/');
	month_string_determine(month_in);
	Print_Character('|');
}

uint8_t poll_switch (void)
{
	uint8_t ret_int = 0;
	switch(PIND & ( (1 << SW_MODE) | (1 << SW_ADJUST) | (1 << SW_RGB) ))
	{
		case 0b00001100:
		ret_int = SW_MODE;
		break;
		case 0b00001010:
		ret_int = SW_RGB;
		break;
		case 0b00000110:
		ret_int = SW_ADJUST;
		break;
		default: break;
	}
	
	return ret_int;
}

uint8_t menu_mode (void)
{
	uint8_t current_state = MENU_ADJUST;
	strcpy(menu_string,ADJUST);
	uint8_t out_confirm_x = 0;
	while(out_confirm_x == 0)
	{
		open_matrix_SPI();
		print_string(menu_string);
		close_matrix_SPI();
		if (poll_switch() == SW_MODE)
		{
			beep(buzzer_state);
			while(poll_switch() == SW_MODE);
			out_confirm_x = 1;
		}
		else if (poll_switch() == SW_ADJUST)
		{
			beep(buzzer_state);
			while(poll_switch() == SW_ADJUST);
			current_state = ladder_string(current_state, up);
		}
		else if (poll_switch() == SW_RGB)
		{
			beep(buzzer_state);
			while(poll_switch() == SW_RGB);
			current_state = ladder_string(current_state, down);
		}
	}
	return current_state;
}

/* Time and date adjust menu mode function */
// uint8_t adjust_mode (uint8_t parameter_in) {
// 	uint8_t ret_value = 0;
// 	out_confirm = 0;
// 	switch(parameter_in)
// 	{
// 		case SEC_state:
// 		while(out_confirm == 0) // Wait until OK
// 		{
// 			open_matrix_SPI();
// 			print_string("<SEC:");
// 			int_to_ascii(ret_value);
// 			Print_Character(buf[0]);
// 			Print_Character(buf[1]);
// 			Print_Character('>');
// 			close_matrix_SPI();
// 			if (poll_switch() == SW_MODE) // Exit from state
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_MODE);
// 				out_confirm = 1;
// 			}
// 			if (poll_switch() == SW_ADJUST)
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_ADJUST);
// 				if (ret_value > 59)  ret_value = 0;
// 				else ret_value++;
// 			}
// 			else if (poll_switch() == SW_RGB) // OK and save appropriate values.
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_RGB);
// 				if (ret_value <= 0)  ret_value = 59;
// 				else ret_value--;
// 			}
// 		}
// 		break;
// 		
// 		case MIN_state:
// 		while(out_confirm == 0) // Wait until OK
// 		{
// 			open_matrix_SPI();
// 			print_string("<MIN:");
// 			int_to_ascii(ret_value);
// 			Print_Character(buf[0]);
// 			Print_Character(buf[1]);
// 			Print_Character('>');
// 			close_matrix_SPI();
// 			if (poll_switch() == SW_MODE) // Exit from state
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_MODE);
// 				out_confirm = 1;
// 			}
// 			else if (poll_switch() == SW_ADJUST)
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_ADJUST);
// 				if (ret_value > 59)  ret_value = 0;
// 				else ret_value++;
// 			}
// 			else if (poll_switch() == SW_RGB) // OK and save appropriate values.
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_RGB);
// 				if (ret_value <= 0)  ret_value = 59;
// 				else ret_value--;
// 			}
// 		}
// 		break;
// 		
// 		case HOUR_state:
// 		while(out_confirm == 0) // Wait until OK
// 		{
// 			open_matrix_SPI();
// 			print_string("<HR: ");
// 			int_to_ascii(ret_value);
// 			Print_Character(buf[0]);
// 			Print_Character(buf[1]);
// 			Print_Character('>');
// 			close_matrix_SPI();
// 			if (poll_switch() == SW_MODE) // Exit from state
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_MODE);
// 				out_confirm = 1;
// 			}
// 			else if (poll_switch() == SW_ADJUST)
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_ADJUST);
// 				if (ret_value > 23)  ret_value = 0;
// 				else ret_value++;
// 			}
// 			else if (poll_switch() == SW_RGB) // OK and save appropriate values.
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_RGB);
// 				if (ret_value <= 0)  ret_value = 23;
// 				else ret_value--;
// 			}
// 		}
// 		break;
// 		
// 		case DAY_state:
// 		while(out_confirm == 0) // Wait until OK
// 		{
// 			buf[0] = '1';
// 			buf[1] = '0';
// 			open_matrix_SPI();
// 			print_string("<DAY:");
// 			int_to_ascii(ret_value);
// 			Print_Character(buf[0]);
// 			Print_Character(buf[1]);
// 			Print_Character('>');
// 			close_matrix_SPI();
// 			if (poll_switch() == SW_MODE) // Exit from state
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_MODE);
// 				out_confirm = 1;
// 			}
// 			else if (poll_switch() == SW_ADJUST)
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_ADJUST);
// 				if (ret_value > 31)  ret_value = 1;
// 				else ret_value++;
// 			}
// 			else if (poll_switch() == SW_RGB) // OK and save appropriate values.
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_RGB);
// 				if (ret_value <= 1)  ret_value = 31;
// 				else ret_value--;
// 			}
// 		}
// 		break;
// 		
// 		case MONTH_state:
// 		while(out_confirm == 0) // Wait until OK
// 		{
// 			buf[0] = '1';
// 			buf[1] = '0';
// 			open_matrix_SPI();
// 			print_string("<MTH:");
// 			int_to_ascii(ret_value);
// 			Print_Character(buf[0]);
// 			Print_Character(buf[1]);
// 			Print_Character('>');
// 			close_matrix_SPI();
// 			if (poll_switch() == SW_MODE) // Exit from state
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_MODE);
// 				out_confirm = 1;
// 			}
// 			else if (poll_switch() == SW_ADJUST)
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_ADJUST);
// 				if (ret_value > 12)  ret_value = 1;
// 				else ret_value++;
// 			}
// 			else if (poll_switch() == SW_RGB) // OK and save appropriate values.
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_RGB);
// 				if (ret_value <= 1)  ret_value = 12;
// 				else ret_value--;
// 			}
// 		}
// 		break;
// 		
// 		case YEAR_state:
// 		while(out_confirm == 0) // Wait until OK
// 		{
// 			open_matrix_SPI();
// 			print_string("<Y:20");
// 			int_to_ascii(ret_value);
// 			Print_Character(buf[0]);
// 			Print_Character(buf[1]);
// 			Print_Character('>');
// 			close_matrix_SPI();
// 			if (poll_switch() == SW_MODE) // Exit from state
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_MODE);
// 				out_confirm = 1;
// 			}
// 			else if (poll_switch() == SW_ADJUST)
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_ADJUST);
// 				if (ret_value > 99)  ret_value = 0;
// 				else ret_value++;
// 			}
// 			else if (poll_switch() == SW_RGB) // OK and save appropriate values.
// 			{
// 				beep(buzzer_state);
// 				while(poll_switch() == SW_RGB);
// 				if (ret_value <= 0)  ret_value = 99;
// 				else ret_value--;
// 			}
// 		}
// 		break;
// 	}
// 	return ret_value;
// }



// #TODO
// Handle LCD refresh
// Switch pressed -> 5 sec to adjustment
// Proper LCD line values

void main() {
	Init_Device();
	struct rtc_time ds1302;
	struct rtc_time *rtc;
	rtc = &ds1302;
	ds1302_init();
	ds1302_update(rtc);
	ds1302_update_time(rtc, HOUR);
	//greetings(rtc->hour);
	
		//ds1302_set_time(rtc, SEC, adjust_mode(SEC_state));
	while (1) {
		Check_IR();
		if (poll_switch()) save_IR_codes_sequence();
		priint_LCD_status();
	}
}

/* Main function */
// int main (void)
// {
// 	
// 	uint32_t total_sec_alarm = 0;
// 	uint32_t total_sec_current_time = 0;
// 	uint32_t dt = 0;
// 	uint8_t ds = 0, dm = 0, dh = 0;
// 	Init_Device();						// Device initialization
// 	//unsigned char sec, temp;			// used variables
// 	uint16_t color_cnt = 0;				// used variable
// 	unsigned long display_cnt = 0;		// used variable
// 	uint8_t RGB_out = PWM_mode;			// Default RGB mode.
// 	uint8_t DISPLAY_out = time_mode;	// Default time displaying mode.
// 	struct rtc_time ds1302;				// DS1302 semi-"class" Definition
// 	struct rtc_time *rtc;
// 	rtc = &ds1302;
// 	ds1302_init();						// DS1302 initialization
// 	ds1302_update(rtc);					// update all fields in the struct
// 	ds1302_update_time(rtc, HOUR);		// Get HOUR value
// 	greetings(rtc->hour);				// Greetings according to the daytime.
// 	while (1)							// Infinite loop
// 	{
// 		if (poll_switch() == SW_MODE) // Check PB state
// 		{
// 			beep(buzzer_state);
// 			while(poll_switch() == SW_MODE);
// 			if (alarm_state) {
// 				alarm_state = false;
// 				hour_alarm = 100;
// 				sec_alarm = 100;
// 				min_alarm = 100;
// 				open_matrix_SPI();
// 				print_string("BEEP:OFF");
// 				close_matrix_SPI();
// 				_delay_ms(1500);
// 			}
// 			else {
// 				switch(menu_mode())
// 				{
// 					case MENU_ADJUST:
// 					ds1302_set_time(rtc, SEC, adjust_mode(SEC_state));
// 					ds1302_set_time(rtc, MIN, adjust_mode(MIN_state));
// 					ds1302_set_time(rtc, HOUR, adjust_mode(HOUR_state));
// 					_delay_ms(50);
// 					ds1302_set_time(rtc, DATE, adjust_mode(DAY_state));
// 					_delay_ms(50);
// 					ds1302_set_time(rtc, MONTH, adjust_mode(MONTH_state));
// 					_delay_ms(50);
// 					break;
// 					
// 					case MENU_COLORS:
// 					RGB_out = menu_mode_color();
// 					break;
// 					
// 					case MENU_DISPLAY:
// 					DISPLAY_out = menu_mode_display();
// 					break;
// 					
// 					case MENU_ALARM:
// 					alarm_state = true;
// 					sec_alarm = adjust_mode(SEC_state);
// 					min_alarm = adjust_mode(MIN_state);
// 					hour_alarm = adjust_mode(HOUR_state);
// 					break;
// 					
// 					case MENU_BUZZER:
// 					buzzer_state = menu_mode_buzzer();
// 					break;
// 					
// 					default: break;
// 				}
// 			}
// 		}
// 		
// 		/* RGB LED Controller */
// 		switch(RGB_out)
// 		{
// 			case R_mode:
// 			stop_timers();
// 			PORTD |=  (1 << RED);
// 			PORTD &= ~(1 << GREEN);
// 			PORTB &= ~(1 << BLUE);
// 			break;
// 			case G_mode:
// 			stop_timers();
// 			PORTD &=  ~(1 << RED);
// 			PORTD |= (1 << GREEN);
// 			PORTB &= ~(1 << BLUE);
// 			break;
// 			case B_mode:
// 			stop_timers();
// 			PORTD &=  ~(1 << RED);
// 			PORTD &= ~(1 << GREEN);
// 			PORTB |= (1 << BLUE);
// 			break;
// 			case PWM_mode:
// 			showRGB(color_cnt);
// 			if (color_cnt == 767) color_cnt = 0;
// 			color_cnt++;
// 			break;
// 			case off_mode:
// 			stop_timers();
// 			PORTD &= ~(1 << RED);
// 			PORTD &= ~(1 << GREEN);
// 			PORTB &= ~(1 << BLUE);
// 			break;
// 			default: break;
// 		}
// 
// 		/* Display Controller */
// 
// 		if (alarm_state) {
// 			open_matrix_SPI();
// 			ds1302_update_time(rtc, SEC);
// 			ds1302_update_time(rtc, MIN);
// 			ds1302_update_time(rtc, HOUR);
// 			total_sec_alarm = (sec_alarm) + (min_alarm*60) + (hour_alarm*3600);
// 			total_sec_current_time = (rtc->second) + ((rtc->minute)*60) + ((rtc->hour)*3600);
// 			dt = total_sec_alarm - total_sec_current_time;
// 			dh = dt / 3600;
// 			dm = (dt / 60) - (dh * 60);
// 			ds = dt - (dm * 60) - (dh * 3600);
// 			if (display_cnt < 400)
// 			{
// 				print_time_string(ds, dm, dh);
// 				display_cnt++;
// 			}
// 			if (display_cnt >= 400)
// 			{
// 				print_string("BEEP IN ");
// 				if (display_cnt == 800) display_cnt = 0;
// 				else display_cnt++;
// 				
// 			}
// 			close_matrix_SPI();
// 		}
// 		else {
// 
// 			switch(DISPLAY_out)
// 			{
// 				case time_mode:
// 				ds1302_update_time(rtc, SEC);
// 				ds1302_update_time(rtc, MIN);
// 				ds1302_update_time(rtc, HOUR);
// 				open_matrix_SPI();
// 				print_time_string(rtc->second,rtc->minute,rtc->hour);
// 				close_matrix_SPI();
// 				break;
// 				
// 				case date_mode:
// 				ds1302_update_time(rtc, DATE);
// 				ds1302_update_time(rtc, MONTH);
// 				open_matrix_SPI();
// 				print_date_string(rtc->date,rtc->month);
// 				close_matrix_SPI();
// 				break;
// 				
// 				case both_mode:
// 				open_matrix_SPI();
// 				if (display_cnt < 400)
// 				{
// 					ds1302_update_time(rtc, SEC);
// 					ds1302_update_time(rtc, MIN);
// 					ds1302_update_time(rtc, HOUR);
// 					print_time_string(rtc->second,rtc->minute,rtc->hour);
// 					display_cnt++;
// 				}
// 				if (display_cnt >= 400)
// 				{
// 					ds1302_update_time(rtc, DATE);
// 					ds1302_update_time(rtc, MONTH);
// 					display_cnt++;
// 					print_date_string(rtc->date,rtc->month);
// 					if (display_cnt == 800) display_cnt = 0;
// 					
// 				}
// 				close_matrix_SPI();
// 				break;
// 				
// 				default: break;
// 			}
// 		}
// 		/* Alarm compare code block */
// 		if ( (hour_alarm == rtc->hour) && (min_alarm == rtc->minute) && (sec_alarm == rtc->second) ) {
// 			alarm_state = false;
// 			alarm_begin();
// 			hour_alarm = 100;
// 			sec_alarm = 100;
// 			min_alarm = 100;
// 		}
// 	}
// 	return 0;
// }