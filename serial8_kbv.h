// this file is only used for 9-bit bidirectional ILI9481

#define USE_SPICLASS
//#define KLUDGE_328P

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
#define xchg8(x)     readbits(8)
#define write8(x)     SPI.transfer(x)
#define write18(x)   { write18_N(x, 1); }
#define WriteCmd(x)  { CD_COMMAND; write8(x); }
#define WriteDat8(x) { CD_DATA; write8(x); }

#include <SPI.h>
static uint8_t spibuf[16];

#define CD_PIN     9
#define CS_PIN     10
#define RESET_PIN  8
#define SD_PIN     4
#define MOSI_PIN   75 //11
#define SCK_PIN    76 //13

#define PIN_LOW(p, b)        digitalWrite(b, LOW)
#define PIN_HIGH(p, b)       digitalWrite(b, HIGH)
#define PIN_OUTPUT(p, b)     pinMode(b, OUTPUT)
#define PIN_INPUT(p, b)      pinMode(b, INPUT_PULLUP)
#define PIN_READ(p, b)       digitalRead(b)

#define SETDDR  { CS_OUTPUT; CD_OUTPUT; RESET_OUTPUT; PIN_HIGH(SD_PORT, SD_PIN); PIN_OUTPUT(SD_PORT, SD_PIN); }
#define INIT()  { CS_IDLE; RESET_IDLE; SETDDR; SPI.begin(CS_PIN); }

#define SDIO_OUTMODE() {MOSI_OUT;SCK_OUT;}

static uint32_t readbits(int8_t bits)
{
	uint32_t ret = 0;
	while (bits > 0) {
	    ret <<= 8;
		ret |= SPI.transfer(0);
		bits -= 8;
	}
	return ret;
}

static inline void write18_N(uint16_t color, int16_t n)
{
	uint8_t r = color >> 8, g = (color >> 5), b = color << 3;
	while (n-- > 0) {
		WriteDat8(r);
		WriteDat8(g);
		WriteDat8(b);
	}
}

static inline void write9_block(uint8_t * block, int16_t n)
{
	while (n-- > 0) {
		WriteDat8(*block++);
	}
}
