#ifndef LCD_H_
#define LCD_H_

#include "main.h"

#define lcd_D7_port     PORTD                   // lcd D7 connection
#define lcd_D7_bit      7
#define lcd_D7_ddr      DDRD

#define lcd_D6_port     PORTC                   // lcd D6 connection
#define lcd_D6_bit      0
#define lcd_D6_ddr      DDRC

#define lcd_D5_port     PORTC                   // lcd D5 connection
#define lcd_D5_bit      1
#define lcd_D5_ddr      DDRC

#define lcd_D4_port     PORTC                   // lcd D4 connection
#define lcd_D4_bit      2
#define lcd_D4_ddr      DDRC

#define lcd_E_port      PORTB                   // lcd Enable pin
#define lcd_E_bit       0
#define lcd_E_ddr       DDRB

#define lcd_RS_port     PORTB                   // lcd Register Select pin
#define lcd_RS_bit      1
#define lcd_RS_ddr      DDRB

// LCD module information 1604A
#define LCD_LINE_1     0x00                    // start of line 1
#define LCD_LINE_2     0x40                    // start of line 2
#define LCD_LINE_3   0x10                  // start of line 3 (20x4)
#define LCD_LINE_4    0x50                  // start of line 4 (20x4)
// LCD instructions
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, 2-line display, 5 x 7 font
#define LCD_SET_CURSOR       0b10000000          // set cursor position

void Init_LCD();
void lcd_write_4(uint8_t);
void lcd_write_instruction_4d(uint8_t);
void lcd_write_character_4d(uint8_t);
void lcd_write_string_4d(char *);
void Init_LCD_4bit(void);
void LCD_logo_display();
void print_LCD_line(char *input_string, uint8_t line_number);
void clear_LCD_line(unsigned char line);
void clear_LCD();
void print_LCD_char(uint8_t ch_in, uint8_t line, uint8_t position);
void print_LCD_segment(char *input_string, uint8_t line, uint8_t segment);
void clear_LCD_segment(uint8_t line, uint8_t segment);
#endif