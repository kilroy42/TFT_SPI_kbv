#define USE_SPICLASS

#if defined(USE_SPICLASS)
#warning Using Arduino SPI methods
#include <SPI.h>                //include before write16() macro
#endif

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
#define xchg8(x)     xchg8_1(x)
#define write16(x)   { write16_N(x, 1); }
#define WriteCmd(x)  { CD_COMMAND; xchg8_1(x); }
#define WriteData(x) { CD_DATA; write16(x); }

#if !defined(USE_SPICLASS)
#include "serial_complex.h"
#else

static uint8_t spibuf[16];

#if defined(ESP8266)
#define CD_PIN     D9
#define CS_PIN     D10
#define RESET_PIN  D8
#define SD_PIN     D4
#define MOSI_PIN   D11
#define SCK_PIN    D13
#else
#define CD_PIN     9
#define CS_PIN     10
#define RESET_PIN  8
#define SD_PIN     4
#define MOSI_PIN   11
#define SCK_PIN    13
#endif

#define SETDDR  { CS_OUTPUT; CD_OUTPUT; RESET_OUTPUT; PIN_HIGH(SD_PORT, SD_PIN); PIN_OUTPUT(SD_PORT, SD_PIN); }
#define INIT()  { CS_IDLE; RESET_IDLE; SETDDR; SPI.begin(); SPI.beginTransaction(settings); }

#define PIN_LOW(p, b)        digitalWrite(b, LOW)
#define PIN_HIGH(p, b)       digitalWrite(b, HIGH)
#define PIN_OUTPUT(p, b)     pinMode(b, OUTPUT)
#define PIN_INPUT(p, b)      pinMode(b, INPUT_PULLUP)
#define PIN_READ(p, b)       digitalRead(b)

static SPISettings settings(40000000, MSBFIRST, SPI_MODE0);

static inline uint8_t xchg8_1(uint8_t x)
{
	return SPI.transfer(x);
}

#define SDIO_INMODE()  SPI.endTransaction(); MOSI_IN;SCK_OUT    //no braces
#define SDIO_OUTMODE() {MOSI_OUT;SCK_OUT;SPI.beginTransaction(settings);}
static uint32_t readbits(uint8_t bits)
{
	uint32_t ret = 0;
	while (bits--) {
		ret <<= 1;
		if (PIN_READ(SPI_PORT, MOSI_PIN))
		    ret++;
		SCK_HI;
		SCK_LO;
	}
	return ret;
}

static inline void write16_N(uint16_t color, int16_t n)
{
#if defined(ESP8266)
    uint8_t hilo[2];
	hilo[0] = color >> 8;
	hilo[1] = color;
	SPI.writePattern(hilo, 2, (uint32_t)n);
#elif defined(BUM__SAM3X8E__)        //this is slower than 8-bit
	while (n-- > 0) {
		SPI.transfer16(color);
	}
#elif defined(TIT__SAM3X8E__)        //
	uint16_t buf[33];
	int16_t count = n;
	if (n <= 0) return;
	SPITransferMode mode = SPI_CONTINUE;
	if (count > 32) count = 32;
//	for (int i = 0; i < count; i++) buf[i] = color;
	while (n > 0) {
		n -= count;
		if (n <= count) {
			count = n;
			mode = SPI_LAST;
		}
	    for (int i = 0; i < count; i++) buf[i] = color;
		SPI.transfer((uint8_t*)buf, count*2, mode);
	}
#else
	uint8_t hi = color >> 8, lo = color;
	while (n-- > 0) {
		SPI.transfer(hi);
		SPI.transfer(lo);
	}
#endif
}

static inline void write8_block(uint8_t * block, int16_t n)
{
#if defined(ESP8266)
	SPI.writeBytes(block, (uint32_t)n);
#else
	SPI.transfer(block, n);
#endif
}
#endif
