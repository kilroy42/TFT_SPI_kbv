#define USE_NOPS_MAX 1

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

static uint8_t spibuf[16];

#if 0
#elif defined(ILI9225_KBV_H_) && defined(__AVR_ATmega328P__)
#define LED_PORT PORTC
#define LED_PIN  PC0
#define CD_PORT PORTC
#define CD_PIN  PC3
#define CS_PORT PORTC
#define CS_PIN  PC5
#define RESET_PORT PORTC
#define RESET_PIN  PC4
#define SD_PORT PORTD
#define SD_PIN  PD4
#define SPI_PORT PORTC
#define MOSI_PIN PC2
#define SCK_PIN  PC1

#define spi_init()
#define SPCRVAL ((1<<SPE)|(1<<MSTR)|(0<<CPHA)|(0<<SPR0))
#define SETDDR  {LED_OUT; SCK_OUT; MOSI_OUT; CD_OUTPUT; RESET_OUTPUT; CS_OUTPUT; }
#define INIT()  { CS_IDLE; RESET_IDLE; LED_HI; SETDDR; spi_init(); }

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168PB__)
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

#define SPCRVAL ((1<<SPE)|(1<<MSTR)|(0<<CPHA)|(0<<SPR0))
#define SETDDR  {DDRB |= (1<<5)|(1<<3)|(1<<2)|(1<<1)|(1<<0); DDRD |= (1<<4); PORTD |= (1<<4); }
#define INIT()  { CS_IDLE; RESET_IDLE; SETDDR; spi_init(); }

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))
#define PIN_INPUT(p, b)      *(&p-1) &= ~(1<<(b))
#define PIN_READ(p, b)       (*(&p-2) & (1<<(b)))

static void spi_init(void)
{
    SPCR = SPCRVAL;
    SPSR = (1 << SPI2X);
    SPSR;
    SPDR;
}

static inline uint8_t xchg8_1(uint8_t c)
{
    SPDR = c;
    while ((SPSR & 0x80) == 0);
    return SPDR;
}

#define SDIO_INMODE() uint8_t spcr = SPCR;SPCR = 0;MOSI_IN      //no braces
#define SDIO_OUTMODE() {MOSI_OUT;SPCR = spcr;}
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

#define NOP1 {asm("nop");}
#define NOP2 {NOP1; NOP1;}
#define NOP4 {NOP2; NOP2;}

#if USE_NOPS_MAX                // -1.39s (8.02s)
static inline void write16_N(uint16_t color, int16_t n)
{
    uint8_t hi = color >> 8, lo = color;
    do {
        SPSR;
        SPDR = hi;
        NOP4;
        NOP4;
        NOP4;
        NOP4;
        SPSR;
        SPDR = lo;
        NOP4;
        NOP4;
        NOP2;
        NOP1;
    } while (--n > 0);
    NOP4;
    NOP2;
    SPSR;
    SPDR;
}

static inline void write8_block(uint8_t * block, int16_t n)
{
    uint8_t c = *block++;
    do {
        SPSR;
        SPDR = c;
        c = *block++;
        NOP4;
        NOP4;
        NOP2;                   //NOP1;
    } while (--n > 0);
    NOP4;
    SPSR;
    SPDR;
}

#else                           // 9.41s

static inline void write16_N(uint16_t color, int16_t n)
{
    uint8_t hi = color >> 8, lo = color;
    SPDR = hi;
    while ((SPSR & 0x80) == 0);
    SPDR = lo;
    while (--n > 0) {
        NOP2;
        while ((SPSR & 0x80) == 0);
        SPDR = hi;
        NOP1;
        while ((SPSR & 0x80) == 0);
        SPDR = lo;
    }
    while ((SPSR & 0x80) == 0);
}

static inline void write8_block(uint8_t * block, int16_t n)
{
    SPDR = *block++;
    while (--n > 0) {
        NOP2;
        while ((SPSR & 0x80) == 0);
        SPDR = *block++;
    }
    while ((SPSR & 0x80) == 0);
}
#endif
#elif defined(__AVR_ATxmega32A4U__)     //3.49s @ 32MHz -O2.
 // 100ns/150ns for ILI9341 W/R cycle.   100ns/200ns for ILI920.  20ns/150ns HX8347
 // Xmega @ 60MHz i.e. 30MHz SCK works with 9341.
#warning Using ATxmega32A4U USART_MSPI
#define CD_PORT VPORT2
#define CD_PIN  1
#define CS_PORT VPORT3
#define CS_PIN  0
#define RESET_PORT VPORT2
#define RESET_PIN  0
#define SD_PORT    PORTC
#define SD_PIN     4
#define SPI_PORT VPORT3 
#define MOSI_PIN 3
#define SCK_PIN  1

#define SPCRVAL (USART_CLK2X_bm | USART_RXEN_bm | USART_TXEN_bm)
#define SETDDR  {PORTCFG.VPCTRLB=PORTCFG_VP13MAP_PORTD_gc | PORTCFG_VP02MAP_PORTC_gc; VPORT3.DIR |= (1<<0)|(1<<1)|(1<<3); VPORT2.DIR |= 0x03; PIN_HIGH(SD_PORT, SD_PIN); SD_PORT.DIR |= (1<<SD_PIN); }
#define INIT()  { CS_IDLE; RESET_IDLE; SETDDR; spi_init(); }

#define PIN_LOW(p, b)        (p).OUT &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p).OUT |= (1<<(b))
#define PIN_OUTPUT(p, b)     (p).DIR |= (1<<(b))
#define PIN_INPUT(p, b)      (p).DIR &= ~(1<<(b))
#define PIN_READ(p, b)       ((p).IN & (1<<(b)))

static inline void spi_init(void)
{
    USARTD0.CTRLB = SPCRVAL;
    USARTD0.CTRLC = USART_CMODE_MSPI_gc | 0x00 | 0x00;  //mode #0 
    //   PORTD.PIN1CTRL |= PORT_INVEN_bm;   //CPOL
    USARTD0.BAUDCTRLA = 0x00;   //F_CPU/2
    USARTD0.BAUDCTRLB = ((0x00 << USART_BSCALE_gp) & USART_BSCALE_gm) | 0x00;
    USARTD0.DATA;
}

static inline uint8_t xchg8_1(uint8_t x)
{
//    USARTD0.STATUS = USART_TXCIF_bm;
    USARTD0.DATA = x;
//    while ((USARTD0.STATUS & USART_TXCIF_bm) == 0);
    while ((USARTD0.STATUS & USART_RXCIF_bm) == 0);
    return USARTD0.DATA;
}

#define SDIO_INMODE() uint8_t spcr = USARTD0.CTRLB;USARTD0.CTRLB = 0;MOSI_IN;SCK_OUT    //no braces
#define SDIO_OUTMODE() {MOSI_OUT;SCK_OUT;USARTD0.CTRLB = spcr;}
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
    USARTD0.DATA = hi;
    while (--n > 0) {
        while ((USARTD0.STATUS & USART_DREIF_bm) == 0);
        USARTD0.DATA = lo;
        while ((USARTD0.STATUS & USART_DREIF_bm) == 0);
        USARTD0.DATA = hi;
    }
    while ((USARTD0.STATUS & USART_DREIF_bm) == 0);
    asm("cli");
    USARTD0.DATA = lo;
    USARTD0.STATUS = USART_TXCIF_bm;
    asm("sei");
    while ((USARTD0.STATUS & USART_TXCIF_bm) == 0);
    while ((USARTD0.STATUS & USART_RXCIF_bm) != 0)
        USARTD0.DATA;
}

static inline void write8_block(uint8_t * block, int16_t n)
{
    USARTD0.DATA = *block++;
    while (--n > 0) {
        while ((USARTD0.STATUS & USART_DREIF_bm) == 0);
        asm("cli");
        USARTD0.DATA = *block++;
        USARTD0.STATUS = USART_TXCIF_bm;
        asm("sei");
    }
    while ((USARTD0.STATUS & USART_TXCIF_bm) == 0);
    while ((USARTD0.STATUS & USART_RXCIF_bm) != 0)
        USARTD0.DATA;
}

//#elif defined(__SAM3X8E__)     //3.49s @ 32MHz -O2.
#else 
// 100ns/150ns for ILI9341 W/R cycle.   100ns/200ns for ILI920.  20ns/150ns HX8347
// Xmega @ 60MHz i.e. 30MHz SCK works with 9341.
#warning Using Arduino SPI methods

#include <SPI.h>

#define CD_PIN  9
#define CS_PIN  10
#define RESET_PIN  8
#define SD_PIN     4
#define MOSI_PIN 11
#define SCK_PIN  13

#define SETDDR  { CS_OUTPUT; CD_OUTPUT; RESET_OUTPUT; PIN_HIGH(SD_PORT, SD_PIN); PIN_OUTPUT(SD_PORT, SD_PIN); }
#define INIT()  { CS_IDLE; RESET_IDLE; SETDDR; SPI.begin(); SPI.beginTransaction(settings); }

#define PIN_LOW(p, b)        digitalWrite(b, LOW)
#define PIN_HIGH(p, b)       digitalWrite(b, HIGH)
#define PIN_OUTPUT(p, b)     pinMode(b, OUTPUT)
#define PIN_INPUT(p, b)      pinMode(b, INPUT)
#define PIN_READ(p, b)       digitalRead(b)

static SPISettings settings(42000000, MSBFIRST, SPI_MODE0);

static inline uint8_t xchg8_1(uint8_t x)
{
	return SPI.transfer(x);
}

#define SDIO_INMODE() SPI.endTransaction();MOSI_IN;SCK_OUT    //no braces
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
#if defined(__SAMD21G18A__)
	uint8_t hi = color >> 8, lo = color;
    Sercom *s = SERCOM1; 
	while (n--) {
	    while (s->SPI.INTFLAG.bit.DRE == 0) ;
        s->SPI.DATA.bit.DATA = hi;
	    while (s->SPI.INTFLAG.bit.DRE == 0) ;
        s->SPI.DATA.bit.DATA = lo;
	}
	while (s->SPI.INTFLAG.bit.TXC == 0) ;
    s->SPI.DATA.bit.DATA;
    s->SPI.DATA.bit.DATA;
#elif defined(___SAM3X8E__)        // this does NOT work
	uint8_t hi = color >> 8, lo = color;
//    Spi *spi = SPI0; 
	    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0) ;
        SPI0->SPI_TDR = hi;
	while (n-- > 1) {
	    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0) ;
        SPI0->SPI_TDR = lo;
	    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0) ;
        SPI0->SPI_TDR = hi;
	}
	    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0) ;
        SPI0->SPI_TDR = lo | SPI_TDR_LASTXFER;
	while ((SPI0->SPI_SR & SPI_SR_TXEMPTY) == 0) ;
	while ((SPI0->SPI_SR & SPI_SR_RDRF) != 0)
        SPI0->SPI_RDR;
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
#if defined(__SAMD21G18A__)
    Sercom *s = SERCOM1; 
	while (n--) {
	    while (s->SPI.INTFLAG.bit.DRE == 0) ;
        s->SPI.DATA.bit.DATA = *block++;
	}
	while (s->SPI.INTFLAG.bit.TXC == 0) ;
    s->SPI.DATA.bit.DATA;
    s->SPI.DATA.bit.DATA;
#else
	SPI.transfer(block, n);
#endif
}

#endif
