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
volatile bool code_received_status = false;
/* END of IR Block */

volatile bool left_monitor_state = OFF;
volatile bool right_monitor_state = OFF;
volatile bool light_state = OFF;
volatile bool sound_state = OFF;
volatile bool power_state = ON;
volatile uint8_t current_rgb_state = RED;
unsigned char keys_array[NUMBER_OF_KEYS];
volatile uint8_t volume_state = 0x00;
volatile uint8_t light_position = 0;

void enable_interrupt(void)
{
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT19);
}

void disable_interrupt(void)
{
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
	if(nec_state != 0){
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
		nec_state = 2;                             // Next state: end of 4.5ms space (start of 562µs pulse)
		return;
		case 2 :                                      // End of 4.5ms space
		if((timer_value > 5000) || (timer_value < 4000)){
			
			nec_state = 0;                             // Reset decoding process
			TCCR1B = 0;                                // Disable Timer1 module
		}
		else
		nec_state = 3;                             // Next state: end of 562µs pulse (start of 562µs or 1687µs space)
		return;
		case 3 :                                      // End of 562µs pulse
		if((timer_value > 700) || (timer_value < 400)){           // Invalid interval ==> stop decoding and reset
			
			TCCR1B = 0;                                // Disable Timer1 module
			nec_state = 0;                             // Reset decoding process
		}
		else
		nec_state = 4;                             // Next state: end of 562µs or 1687µs space
		return;
		case 4 :                                      // End of 562µs or 1687µs space
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
		nec_state = 3;                               // Next state: end of 562µs pulse (start of 562µs or 1687µs space)
	}
}

void initInterrupts() {
	TCCR1A = 0;
	TCCR1B = 0;                                    // Disable Timer1 module
	TCNT1  = 0;                                    // Set Timer1 preload value to 0 (reset)
	TIMSK1 = 1;                                    // enable Timer1 overflow interrupt
	enable_interrupt();
}

void left_monitor(bool statex){
	if (statex) {
		left_monitor_state = true;
		PORTD |= (1 << MT1_EN);
	}
	else {
		left_monitor_state = false;
		PORTD &= ~(1 << MT1_EN);
	}
}

void right_monitor(bool statex) {
	if (statex) {
		right_monitor_state = true; 
		PORTD |= (1 << MT0_EN);
	}
	else {
		right_monitor_state = false;
		PORTD &= ~(1 << MT0_EN);
	}
}

void Init_light(void) {
	DDRC |= (1 << LIGHT_nINC) | (1 << LIGHT_UnD) | (1 << LIGHT_nCS);
	LIGHT_PORT |= (1 << LIGHT_nINC);
	LIGHT_PORT |= (1 << LIGHT_UnD);
	LIGHT_PORT |= (1 << LIGHT_nCS);
}

void Init_UI(void) {
	DDRD |= (1 << MT1_EN) | (1 << MT0_EN);
}

void Init_RGB(void) {
	DDRB |= (1 << LED_R);
	DDRB |= (1 << LED_G);
	DDRD |= (1 << LED_B);	
	set_LED(RED);
	_delay_ms(1000);
	set_LED(GREEN);
	_delay_ms(1000);
}


// void Check_Circuits(void) {
// 	writeString("\r\nHello\r\n");
// 	_delay_ms(1000);
// 	left_monitor(ON);
// 	_delay_ms(1000);
// 	right_monitor(ON);
// 	_delay_ms(1000);
// 	//LED_out(GREEN | RED | BLUE);
// 	_delay_ms(1000);
// 	SPI_write_16bit(WIPER0, 0x00);
// 	SPI_write_16bit(WIPER1, 0x00);
// 	_delay_ms(1000);
// 	PORTB |= (1 << 6);
// 	_delay_ms(1000);
// 	PORTB |= (1 << 7);
// 	_delay_ms(1000);
// 	PORTD |= (1 << 5);
// }

void light_up(void) {
LIGHT_PORT &= ~(1 << LIGHT_UnD);
LIGHT_PORT &= ~(1 << LIGHT_nINC);
_delay_us(100);
LIGHT_PORT |= (1 << LIGHT_nINC);
_delay_us(100);
}

void light_down(void) {
	LIGHT_PORT |= (1 << LIGHT_UnD);
	LIGHT_PORT &= ~(1 << LIGHT_nINC);
	_delay_us(100);
	LIGHT_PORT |= (1 << LIGHT_nINC);
	_delay_us(100);
}



void SPI_write_16bit(uint8_t address_in, uint8_t data_in)
{
	SPDR = address_in;
	while(!(SPSR & (1<<SPIF)));
	SPDR = data_in;
	while(!(SPSR & (1<<SPIF)));
	SPI_PORT |= (1<<SPI_CS);
	SPI_PORT &= ~(1<<SPI_CS);
}

uint8_t SPI_read_16bit(uint8_t address_in)
{
	//SPDR = ((address_in << 4) & 0xF0) || SPI_read_command;
	SPDR = 0b00001100;
	while(!(SPSR & (1<<SPIF)));
	SPDR = 0;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
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

unsigned char get_IR_code() {
	if(nec_ok) {
		nec_ok = 0;                                  // Reset decoding process
		nec_state = 0;
		TCCR1B = 0;                                  // Disable Timer1 module
		address = nec_code >> 16;
		command = nec_code >> 8;
		inv_command = nec_code;
		enable_interrupt();
		set_LED(BLUE);
		_delay_ms(100);
		set_LED(GREEN);
		return inv_command;
	}
	return 0;
}

void Check_IR() {
	volatile uint8_t int_LCD = 0;
	char buff_LCD[NUM_OF_LCD_CHARS];
	if(nec_ok) {
		nec_ok = 0;                                  // Reset decoding process
		nec_state = 0;
		TCCR1B = 0;                                  // Disable Timer1 module
		address = nec_code >> 16;
		command = nec_code >> 8;
		inv_command = nec_code;
		if (power_state == OFF && (inv_command != keys_array[KEY_POWER_ADDR]));
		else {
			if (inv_command == keys_array[KEY_VOL_DOWN_ADDR]) {
				if (volume_state < VOLUME_SMALL_STEP){
					volume_state = volume_state;
					set_LED(RED);
					_delay_ms(100);
					set_LED(GREEN);
				}
				else {
					volume_state -= VOLUME_SMALL_STEP;
					set_LED(BLUE);
					_delay_ms(100);
					set_LED(GREEN);
				}
				SPI_write_16bit(WIPER0, volume_state);
				SPI_write_16bit(WIPER1, volume_state);
				clear_LCD_line(LCD_LINE_1);
				int_LCD = (volume_state * 100) / VOLUME_LIMIT;
				snprintf(buff_LCD, NUM_OF_LCD_CHARS, "Volume: %d     ", int_LCD);
				print_LCD_line(buff_LCD, LCD_LINE_1);
			}
			else if (inv_command == keys_array[KEY_VOL_UP_ADDR]) {
				if (volume_state > VOLUME_LIMIT) {
					volume_state = VOLUME_LIMIT;
					set_LED(RED);
					_delay_ms(100);
					set_LED(GREEN);
				}
				else {
					volume_state += VOLUME_SMALL_STEP;
					set_LED(BLUE);
					_delay_ms(100);
					set_LED(GREEN);
				}
				SPI_write_16bit(WIPER0, volume_state);
				SPI_write_16bit(WIPER1, volume_state);
				clear_LCD_line(LCD_LINE_1);
				int_LCD = (volume_state * 100) / VOLUME_LIMIT;
				snprintf(buff_LCD, NUM_OF_LCD_CHARS, "Volume: %d     ", int_LCD);
				print_LCD_line(buff_LCD, LCD_LINE_1);
			}
			else if (inv_command == keys_array[KEY_ADJ_R_ADDR]) {
				if (volume_state > VOLUME_LIMIT) {
					volume_state = volume_state;
					set_LED(RED);
					_delay_ms(100);
					set_LED(GREEN);
				}
				else {
					volume_state += VOLUME_LARGE_STEP;
					set_LED(BLUE);
					_delay_ms(100);
					set_LED(GREEN);
				}
				SPI_write_16bit(WIPER0, volume_state);
				SPI_write_16bit(WIPER1, volume_state);
				clear_LCD_line(LCD_LINE_1);
				int_LCD = (volume_state * 100) / VOLUME_LIMIT;
				snprintf(buff_LCD, NUM_OF_LCD_CHARS, "Volume: %d     ", int_LCD);
				print_LCD_line(buff_LCD, LCD_LINE_1);
			}
			else if (inv_command == keys_array[KEY_ADJ_L_ADDR]) {
				if (volume_state < VOLUME_LARGE_STEP){
					volume_state = volume_state;
					set_LED(RED);
					_delay_ms(100);
					set_LED(GREEN);
				}
				else {
					volume_state -= VOLUME_LARGE_STEP;
					set_LED(BLUE);
					_delay_ms(100);
					set_LED(GREEN);
				}
				SPI_write_16bit(WIPER0, volume_state);
				SPI_write_16bit(WIPER1, volume_state);
				clear_LCD_line(LCD_LINE_1);
				int_LCD = (volume_state * 100) / VOLUME_LIMIT;
				snprintf(buff_LCD, NUM_OF_LCD_CHARS, "Volume: %d     ", int_LCD);
				print_LCD_line(buff_LCD, LCD_LINE_1);
			}
			else if (inv_command == keys_array[KEY_CH_UP_ADDR]) {
				if (light_position < LIGHT_POSITION_MAX) {
					light_up();
					light_position++;
					light_state = ON;
					 set_LED(BLUE);
					 _delay_ms(100);
					 set_LED(GREEN);
				}
				else {
					set_LED(RED);
					_delay_ms(100);
					set_LED(GREEN);
				}
				clear_LCD_line(LCD_LINE_2);
				int_LCD = (light_position * 100) / 5;
				snprintf(buff_LCD, NUM_OF_LCD_CHARS, "Light: %d    ", int_LCD);
				print_LCD_line(buff_LCD, LCD_LINE_2);
			}
				
			else if (inv_command == keys_array[KEY_CH_DOWN_ADDR]) {
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
				clear_LCD_line(LCD_LINE_2);
				int_LCD = (light_position * 100) / LIGHT_POSITION_MAX;
				snprintf(buff_LCD, NUM_OF_LCD_CHARS, "Light: %d    ", int_LCD);
				print_LCD_line(buff_LCD, LCD_LINE_2);
			}	
			
			else if (inv_command == keys_array[KEY_A_ADDR]) {
				if (left_monitor_state == ON) {
					left_monitor(OFF);
					print_LCD_line("    MTR A: Off  ", LCD_LINE_3);
				}
				else {
					left_monitor(ON);
					print_LCD_line("    MTR A: On   ", LCD_LINE_3);
				}
				set_LED(BLUE);
				_delay_ms(100);
				set_LED(GREEN);
			}
			
			else if (inv_command == keys_array[KEY_B_ADDR]) {
				if (right_monitor_state == ON) {
					right_monitor(OFF);
					print_LCD_line("    MTR B: Off  ", LCD_LINE_4);
				}
				else {
					right_monitor(ON);
					print_LCD_line("    MTR B: On   ", LCD_LINE_4);
				}
				set_LED(BLUE);
				_delay_ms(100);
				set_LED(GREEN);
			}
			
			else if (inv_command == keys_array[KEY_C_ADDR]) {
				if (light_state == ON) {
					disable_light();
					print_LCD_line("Light: Off      ", LCD_LINE_2);
				}
				else {
					enable_light();
					print_LCD_line("Light: Full     ", LCD_LINE_2);
				}
				set_LED(BLUE);
				_delay_ms(100);
				set_LED(GREEN);
			}
				
			else if (inv_command == keys_array[KEY_POWER_ADDR]) {
				if (power_state == ON) disable_all();
				else enable_all();
			}
			
			else if (inv_command == keys_array[KEY_MUTE_ADDR]) {
				if (sound_state == ON) {
					set_volume(0);
					sound_state = OFF;
					print_LCD_line("Volume: Muted   ", LCD_LINE_1);
				}
				else {
					set_volume(volume_state);
					sound_state = ON;
					clear_LCD_line(LCD_LINE_1);
					int_LCD = (volume_state * 100) / VOLUME_LIMIT;
					snprintf(buff_LCD, NUM_OF_LCD_CHARS, "Volume: %d     ", int_LCD);
					print_LCD_line(buff_LCD, LCD_LINE_1);
				}
				set_LED(BLUE);
				_delay_ms(100);
				set_LED(GREEN);
			}						
		}
		enable_interrupt();
	}
}

void disable_all() {
	power_state = OFF;
	left_monitor(OFF);
	right_monitor(OFF);
	disable_light();
	set_volume(0);
	set_LED(RED);
	disable_LCD();
}

void enable_all() {
	power_state = ON;
	left_monitor(ON);;
	right_monitor(ON);
	enable_light();
	set_volume(volume_state);
	set_LED(GREEN);
	enable_LCD();
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
	light_position = LIGHT_POSITION_MAX;
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

void time_led(uint8_t led_type) {
	uint8_t rgb_state_imm = current_rgb_state;
	set_LED(led_type);
	_delay_ms(100);
	set_LED(rgb_state_imm);
}


void Init_switch_LCD_power() {
	DDRD |= (1 << PB) | (1 << LCD_EN);
}

void Init_Device() {
	Init_switch_LCD_power();
	Init_RGB();
	set_LED(GREEN);
	Init_light();
	initInterrupts();
	initUART();
	Init_UI();
	Init_SPI();
	Init_LCD();
	enable_LCD();
	sei();
}

void display_main_UI_LCD() {
	clear_LCD();
	print_LCD_line("Volume: <ADJ>   ", LCD_LINE_1);
	print_LCD_line("Light:  <ADJ>   ", LCD_LINE_2);
	print_LCD_line("    MTR A: <ADJ>", LCD_LINE_3);
	print_LCD_line("    MTR B: <ADJ>", LCD_LINE_4);	
}

void LCD_logo_display() {
	print_LCD_line("Smart Workbench+", LCD_LINE_1);
	print_LCD_line("   Rev: V2.0    ", LCD_LINE_2);
	print_LCD_line("Configurable IR ", LCD_LINE_3);
	print_LCD_line("Based Workbench ", LCD_LINE_4);
}

void enable_LCD() {
	PORTD |= (1 << LCD_EN);
	Init_LCD_4bit();
	Init_light_state();
	LCD_logo_display();
	_delay_ms(3000);
	display_main_UI_LCD();
}

void disable_LCD() {
	PORTD &= ~(1 << LCD_EN);
}

void load_IR_codes_from_EEPROM() {
	for (unsigned char ptrEEPROM = 0; ptrEEPROM < 12; ptrEEPROM++) {
		keys_array[ptrEEPROM] = eeprom_read_byte(ptrEEPROM);
	}
}

void init_IR_pairing_sequence() {
	uint8_t IR_pairing_state = 0;
	uint8_t received_IR_code = 0;
	bool out_confirm = false;
	clear_LCD();
	print_LCD_line("Press the Left  ", LCD_LINE_1);
	print_LCD_line("Monitor Switch  ", LCD_LINE_2);
	print_LCD_line("Key...          ", LCD_LINE_3);
	initInterrupts();
	while (!out_confirm) {
		if (poll_switch())  {
			switch_pressed();
			display_main_UI_LCD();
			break;
		}
		received_IR_code = get_IR_code();
		switch(IR_pairing_state) {
			case 0: 
				if (received_IR_code != 0) {
					eeprom_write_byte(( uint8_t *)KEY_A_ADDR, received_IR_code); 
					IR_pairing_state = 1;
					print_LCD_line("Press the Right ", LCD_LINE_1);
					print_LCD_line("Monitor Key...  ", LCD_LINE_2);
				}
				break;
			
			case 1:
				if (received_IR_code != 0) {
					eeprom_write_byte(( uint8_t *)KEY_B_ADDR, received_IR_code);
					IR_pairing_state = 2;
					print_LCD_line("Press the Light ", LCD_LINE_1);
					print_LCD_line("Switch Key...   ", LCD_LINE_2);
				}
				break;

			case 2:
				if (received_IR_code != 0) {
					eeprom_write_byte(( uint8_t *)KEY_C_ADDR, received_IR_code);
					IR_pairing_state = 3;
					print_LCD_line("Press the Mute  ", LCD_LINE_1);
					print_LCD_line("Switch Key...   ", LCD_LINE_2);
				}
				break;	

			case 3:
				if (received_IR_code != 0) {
					eeprom_write_byte(( uint8_t *)KEY_MUTE_ADDR, received_IR_code);
					IR_pairing_state = 4;
					print_LCD_line("Press the Power ", LCD_LINE_1);
					print_LCD_line("Switch Key...   ", LCD_LINE_2);
				}
				break;	
				
			case 4:
				if (received_IR_code != 0) {
					eeprom_write_byte(( uint8_t *)KEY_POWER_ADDR, received_IR_code);
					IR_pairing_state = 5;
					print_LCD_line("Press the Volume", LCD_LINE_1);
					print_LCD_line("Up Key...       ", LCD_LINE_2);  // eeprom_read_byte(STORED_CONNECTION_STATE_ADDRESS);
				}
				break;	
				
			case 5:
				if (received_IR_code != 0) {
					eeprom_write_byte(( uint8_t *)KEY_VOL_UP_ADDR, received_IR_code);
					IR_pairing_state = 6;
					print_LCD_line("Press the Volume", LCD_LINE_1);
					print_LCD_line("Down Key...     ", LCD_LINE_2);  // eeprom_read_byte(STORED_CONNECTION_STATE_ADDRESS);
				}
				break;	

			case 6:
				if (received_IR_code != 0) {
					eeprom_write_byte(( uint8_t *)KEY_VOL_DOWN_ADDR, received_IR_code);
					IR_pairing_state = 7;
					print_LCD_line("Press the Light ", LCD_LINE_1);
					print_LCD_line("Up Key...       ", LCD_LINE_2);  // eeprom_read_byte(STORED_CONNECTION_STATE_ADDRESS);
				}
				break; 
				
			case 7:
				if (received_IR_code != 0) {
					eeprom_write_byte(( uint8_t *)KEY_CH_UP_ADDR, received_IR_code);
					IR_pairing_state = 8;
					print_LCD_line("Press the Light ", LCD_LINE_1);
					print_LCD_line("Down Key...     ", LCD_LINE_2);  // eeprom_read_byte(STORED_CONNECTION_STATE_ADDRESS);
				}
				break; 

			case 8:
				if (received_IR_code != 0) {
					eeprom_write_byte(( uint8_t *)KEY_CH_DOWN_ADDR, received_IR_code);
					IR_pairing_state = 9;
					print_LCD_line("Press the Volume", LCD_LINE_1);
					print_LCD_line("Fast Up Key...  ", LCD_LINE_2);  // eeprom_read_byte(STORED_CONNECTION_STATE_ADDRESS);
				}
				break;
				
			case 9:
				if (received_IR_code != 0) {
					eeprom_write_byte(( uint8_t *)KEY_ADJ_R_ADDR, received_IR_code);
					IR_pairing_state = 10;
					print_LCD_line("Press the Volume", LCD_LINE_1);
					print_LCD_line("Fast Down Key...  ", LCD_LINE_2);  // eeprom_read_byte(STORED_CONNECTION_STATE_ADDRESS);
				}
				break;
				
			case 10:
				if (received_IR_code != 0) {
					IR_pairing_state = 0;
					eeprom_write_byte(( uint8_t *)KEY_ADJ_L_ADDR, received_IR_code);
					out_confirm = true;
					clear_LCD();
					print_LCD_line("Remote is Ready!", LCD_LINE_2);
					_delay_ms(2000);
					load_IR_codes_from_EEPROM();
					enable_interrupt();
				}
				break;
			
		}
	}
}

void switch_pressed() {
	clear_LCD();
	print_LCD_line("Press the button", LCD_LINE_1);
	print_LCD_line("again to proceed", LCD_LINE_2);
	print_LCD_line("to IR pairing   ", LCD_LINE_3);
	print_LCD_line("in X seconds    ", LCD_LINE_4);
	for (uint8_t iPtr = 0; iPtr < 5; iPtr++) {
		print_LCD_char((5 - iPtr + 0x30), LCD_LINE_4, 3);
		_delay_ms(1000);
		if (poll_switch()) init_IR_pairing_sequence();
	}
	display_main_UI_LCD();
}

bool poll_switch() {
	if (!(PIND & (1 << PB))) {
		while(!(PIND & (1 << PB)));
		return true;
	}
	else return false;
}

int main() {
	Init_Device();
	DDRD &= ~(1 << IR_OUT);
	load_IR_codes_from_EEPROM();
	while (1) {
		Check_IR();
		if (poll_switch()) switch_pressed();
	}
	return 0;
}