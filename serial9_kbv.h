// this file is only used for 9-bit bidirectional ILI9481

#define USE_SPICLASS
#define KLUDGE_328P

#define CD_COMMAND PIN_LOW(CD_PORT, CD_PIN)
#define CD_DATA    PIN_HIGH(CD_PORT, CD_PIN)
#define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)
#define CS_ACTIVE  PIN_LOW(CS_PORT, CS_PIN);
#define CS_IDLE    PIN_HIGH(CS_PORT, CS_PIN);
#define CS_OUTPUT  PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)
#define SD_ACTIVE  PIN_LOW(SD_PORT, SD_PIN)
#define SD_IDLE    PIN_HIGH(SD_PORT, SD_PIN)
#define SD_OUTPUT  PIN_OUTPUT(SD_PORT, SD_PIN)
 // bit-bang macros for SDIO
#define SCK_LO     PIN_LOW(SPI_PORT, SCK_PIN)
#define SCK_HI     PIN_HIGH(SPI_PORT, SCK_PIN)
#define SCK_OUT    PIN_OUTPUT(SPI_PORT, SCK_PIN)
#define MOSI_LO    PIN_LOW(SPI_PORT, MOSI_PIN)
#define MOSI_HI    PIN_HIGH(SPI_PORT, MOSI_PIN)
#define MOSI_OUT   PIN_OUTPUT(SPI_PORT, MOSI_PIN)
#define MOSI_IN    PIN_INPUT(SPI_PORT, MOSI_PIN)
#define LED_LO     PIN_LOW(LED_PORT, LED_PIN)
#define LED_HI     PIN_HIGH(LED_PORT, LED_PIN)
#define LED_OUT    PIN_OUTPUT(LED_PORT, LED_PIN)

#define wait_ms(ms)  delay(ms)
#define xchg8(x)     readbits(8);
#define write24(x)   { write24_N(x, 1); }
#define WriteCmd(x)  { SDIO_OUTMODE(); MOSI_LO; SCK_HI; SCK_LO; write_8(x); }
#define WriteDat8(x) { MOSI_HI; SCK_HI; SCK_LO; write_8(x); }

#if !defined(USE_SPICLASS)
#include "serial_complex.h"
#else
#warning Using Arduino SPI methods

#include <SPI.h>
static uint8_t spibuf[16];

#if defined(KLUDGE_328P)
#define CD_PORT PORTB
#define CD_PIN  PB1
#define CS_PORT PORTB
#define CS_PIN  PB2
#define RESET_PORT PORTB
#define RESET_PIN  PB0
#define SD_PORT PORTD
#define SD_PIN  PD4
#define SPI_PORT PORTB
#define MOSI_PIN PB3
#define SCK_PIN  PB5

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))
#define PIN_INPUT(p, b)      *(&p-1) &= ~(1<<(b))
#define PIN_READ(p, b)       (*(&p-2) & (1<<(b)))
#else
#define CD_PIN     9
#define CS_PIN     10
#define RESET_PIN  8
#define SD_PIN     4
#define MOSI_PIN   11
#define SCK_PIN    13

#define PIN_LOW(p, b)        digitalWrite(b, LOW)
#define PIN_HIGH(p, b)       digitalWrite(b, HIGH)
#define PIN_OUTPUT(p, b)     pinMode(b, OUTPUT)
#define PIN_INPUT(p, b)      pinMode(b, INPUT_PULLUP)
#define PIN_READ(p, b)       digitalRead(b)
#endif

#define SETDDR  { CS_OUTPUT; CD_OUTPUT; RESET_OUTPUT; PIN_HIGH(SD_PORT, SD_PIN); PIN_OUTPUT(SD_PORT, SD_PIN); }
#define INIT()  { CS_IDLE; RESET_IDLE; SETDDR; }

#define SDIO_INMODE()  MOSI_IN;SCK_OUT    //no braces
#define SDIO_OUTMODE() {MOSI_OUT;SCK_OUT;}

static inline void write_8(uint8_t val)
{
#if defined(KLUDGE_328P)                // -90.0 sec
    SPCR = (1<<SPE)|(1<<MSTR);
	SPSR = (1<<SPI2X);
	SPDR = val;
	while((SPSR & 0x80) == 0); 
	SPCR = 0;
#else
    for (uint8_t i = 0; i < 8; i++) {   //send command
        if (val & 0x80) MOSI_HI;
	    else MOSI_LO;
		SCK_HI;
		SCK_LO;
        val <<= 1;
    }
#endif
}

static uint32_t readbits(uint8_t bits)
{
	uint32_t ret = 0;
    SDIO_INMODE();
	while (bits--) {
		ret <<= 1;
		if (PIN_READ(SPI_PORT, MOSI_PIN))
		    ret++;
		SCK_HI;
		SCK_LO;
	}
	return ret;
}

static inline void write24_N(uint16_t color, int16_t n)
{
#if defined(KLUDGE_328P)                // -15.5 sec
	uint8_t r = (color >> 9) | 0x80, g = (color >> 5) | 0x40, b = color | 0x20;
	while (n-- > 0) {
    SPCR = (1<<SPE)|(1<<MSTR);
	SPSR = (1<<SPI2X);
	SPDR = r;
	while((SPSR & 0x80) == 0); 
	SPDR = g;
	while((SPSR & 0x80) == 0); 
	SPDR = b;
	while((SPSR & 0x80) == 0); 
	SPCR = 0;
    SCK_HI; SCK_LO;SCK_HI; SCK_LO;SCK_HI; SCK_LO;	
	}
#else
	uint8_t r = color >> 8, g = (color >> 3), b = color << 3;
	while (n-- > 0) {
		WriteDat8(r);
		WriteDat8(g);
		WriteDat8(b);
	}
#endif
}

static inline void write8_block(uint8_t * block, int16_t n)
{
	while (n-- > 0) {
		WriteDat8(*block++);
	}
}
#endif
