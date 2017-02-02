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

#if defined(__STM32F1__) && defined(ARDUINO_STM_NUCLEO_F103RB) // Uno Shield on NUCLEO-F103
#warning Uno Shield on NUCLEO-F103 REGS
// be wise to clear all four mode bits properly.
#define GROUP_MODE(port, reg, mask, val)  {port->regs->reg = (port->regs->reg & ~(mask)) | ((mask)&(val)); }
#define GP_OUT(port, reg, mask)           GROUP_MODE(port, reg, mask, 0x33333333)
#define GP_INP(port, reg, mask)           GROUP_MODE(port, reg, mask, 0x44444444)

  #define CD_PORT GPIOC
  #define CD_PIN  7
  #define CS_PORT GPIOB
  #define CS_PIN  6
  #define RESET_PORT GPIOA
  #define RESET_PIN  9
  #define SD_PORT GPIOB
  #define SD_PIN  5
  #define MOSI_PORT GPIOA
  #define MOSI_PIN  7
  #define SCK_PORT GPIOA
  #define SCK_PIN  5
  
  #define SPI_PORT GPIOA

#define PIN_HIGH(port, pin)   (port)->regs->BSRR = (1<<(pin))
#define PIN_LOW(port, pin)    (port)->regs->BSRR = (1<<((pin)+16))
//#define PIN_LOW(port, pin)    (port)->regs->ODR &= ~(1<<(pin))
#define PIN_OUTPUT(port, pin) gpio_set_mode(port, pin, GPIO_OUTPUT_PP)   //50MHz push-pull only 0-7
#define PIN_INPUT(port, pin)  gpio_set_mode(port, pin, GPIO_INPUT_FLOATING)   //digital input 
#define PIN_READ(port, pin)   ((port)->regs->IDR & (1<<(pin)))
#else
#error mcu unsupported
#endif

#define xchg8(x)     readbits(8)
#define WriteCmd(x)  { SDIO_OUTMODE(); MOSI_LO; SCK_HI; SCK_HI; SCK_LO; write_8(x); }
#define WriteDat8(x) { MOSI_HI; SCK_HI; SCK_HI; SCK_LO; write_8(x); }
#define INIT()  { CS_IDLE; RESET_IDLE; SETDDR; }
#define SDIO_INMODE()  MOSI_IN;SCK_OUT    //no braces
#define SDIO_OUTMODE() {MOSI_OUT;SCK_OUT;}

#define wait_ms(ms)  delay(ms)
#define write16(x)   { write16_N(x, 1); }
#define write24(x)   { write24_N(x, 1); }
#define WriteData(x) { write16(x); }

#define SETDDR  { CS_OUTPUT; CD_OUTPUT; RESET_OUTPUT; PIN_HIGH(SD_PORT, SD_PIN); PIN_OUTPUT(SD_PORT, SD_PIN); }

static uint8_t spibuf[16];

static inline void write_8(uint8_t val)
{
    //Saleae is unhappy with 41.67ns SCK pulses.   It misses them
	//unrolling works fine for the TFT.  Loop versions give 83ns SCK
#if 0
    uint8_t mask = 0x80;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; val <<= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; val <<= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; val <<= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; val <<= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; val <<= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; val <<= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; val <<= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; val <<= 1; SCK_LO;
#elif 0
    uint8_t mask = 0x80;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; mask >>= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; mask >>= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; mask >>= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; mask >>= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; mask >>= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; mask >>= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; mask >>= 1; SCK_LO;
    if (val & mask) MOSI_HI; else MOSI_LO; SCK_HI; mask >>= 1; SCK_LO;
#elif 1
    for (uint8_t mask = 0x80; mask; ) {   //send command
        if (val & mask) MOSI_HI;
	    else MOSI_LO;
		SCK_HI;
		mask >>= 1;
		SCK_LO;
    }
#else
    for (uint32_t i = 0; i < 8; i++) {   //send command
        if (val & 0x80) MOSI_HI;
	    else MOSI_LO;
		SCK_HI;
        val <<= 1;
		SCK_LO;
    }
#endif
}

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
	uint8_t hi = color >> 8, lo = color;
	while (n-- > 0) {
		WriteDat8(hi);
		WriteDat8(lo);
	}
}

static inline void write24_N(uint16_t color, int16_t n)
{
	uint8_t r = color >> 8, g = (color >> 3), b = color << 3;
	while (n-- > 0) {
		WriteDat8(r);
		WriteDat8(g);
		WriteDat8(b);
	}
}

static inline void write8_block(uint8_t * block, int16_t n)
{
    while (n-- > 0) WriteDat8(*block++);
}
