#include "uart.h"

// Debug Mode; comment out on Release
//#define _DEBUG			0

/*! \brief Configures baud rate (refer datasheet) */
void initUART() {
	// Not necessary; initialize anyway
	DDRD |= 1 << PORTD1;

	// Set baud rate; lower byte and top nibble
	UBRR0H = ((_UBRR) & 0xF00);
	UBRR0L = (uint8_t) ((_UBRR) & 0xFF);

	TX_START();
	RX_START();

	// Set frame format = 8-N-1
	UCSR0C = (_DATA << UCSZ00);
}

void putByte(unsigned char data)
{
	// Stay here until data buffer is empty
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = (unsigned char) data;

}

/*! \brief Writes an ASCII string to the TX buffer */
void writeString(char *str)
{
	while (*str != '\0')
	{
		putByte(*str);
		++str;
	}
}

//#if _DEBUG
