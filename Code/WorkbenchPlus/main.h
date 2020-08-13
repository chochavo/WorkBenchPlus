#ifndef MAIN_H_
#define MAIN_H_

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "uart.h"
#include "lcd.h"

#define MT1_EN PORTD2
#define MT0_EN PORTD4
#define LED_R PORTB6
#define LED_G PORTB7
#define LED_B PORTD5
#define PB PORTD0
#define LCD_EN PORTD6
#define ON 1
#define OFF 0

#define RED 0x04
#define BLUE	  0x02
#define GREEN  0x01

#define VOLUME_LIMIT 200
#define NUMBER_OF_KEYS 12
#define VOLUME_SMALL_STEP 2
#define VOLUME_LARGE_STEP 16
#define LIGHT_POSITION_MAX 5
#define NUM_OF_LCD_CHARS 16

// EEPROM ADDRESSES //
#define KEY_ADJ_R_ADDR 0x00
#define KEY_A_ADDR 0x01
#define KEY_B_ADDR 0x02
#define KEY_C_ADDR 0x03
#define KEY_MUTE_ADDR 0x04
#define KEY_POWER_ADDR 0x05
#define KEY_PLAY_ADDR 0x06
#define KEY_VOL_UP_ADDR 0x07
#define KEY_VOL_DOWN_ADDR 0x08
#define KEY_CH_UP_ADDR 0x09
#define KEY_CH_DOWN_ADDR 0x0A
#define KEY_ADJ_L_ADDR 0x0B
//

// #define KEY_A 0x97
// #define KEY_B 0x67
// #define KEY_C 0x4F
// #define KEY_MUTE 0x5D
// #define KEY_POWER 0xAD
// #define KEY_PLAY 0xBD
// #define KEY_VOL_UP 0x3D
// #define KEY_VOL_DOWN 0xFD
// #define KEY_CH_UP 0x1D
// #define KEY_CH_DOWN 0x9D
// #define KEY_ADJ_L 0x57
// #define KEY_ADJ_R 0x6F

// IR State Machine
#define INIT 10
#define pulse_9ms 11
#define pulse_4_5ms 12
#define pulse_562_us 13
#define pulse_1687_us 14
#define END_TRANSMISSION 15
#define IR_OUT PORTD3 
#define SPI_PORT PORTB
#define SPI_DDR  DDRB
#define SPI_CS   PORTB2

#define LIGHT_nINC PORTC5
#define LIGHT_UnD  PORTC4
#define LIGHT_nCS  PORTC3
#define LIGHT_PORT PORTC

#define SPI_read_command	  0b00001100
#define SPI_write_command	  0b00000000
#define SPI_increment_command 0b00000100
#define SPI_decrement_command 0b00001000

#define WIPER0 0b00010001
#define WIPER1 0b00010010

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
void disable_LCD();
void Init_switch_LCD_power();
void LCD_logo_display();
void display_main_UI_LCD();
void enable_LCD();
void disable_LCD();
void init_IR_pairing_sequence();
bool poll_switch();
void switch_pressed();
/////////////////////////////////////


#endif /* MAIN_H_ */