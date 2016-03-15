/*
 * tftspi_avr.h
 *
 * Created: 10/11/2014 20:48:01
 *  Author: David Prentice
 */ 


#ifndef TFTSPI_AVR_H_
#define TFTSPI_AVR_H_

#include <avr/io.h>

#if 0
#elif defined(__AVR_ATxmega128A1__)     //3.49s @ 32MHz -O2
  #define CD_PORT VPORT2
  #define CD_PIN  1
  #define CS_PORT VPORT3
  #define CS_PIN  4
  #define RESET_PORT VPORT2
  #define RESET_PIN  0
#define SPCRVAL (USART_CLK2X_bm | USART_RXEN_bm | USART_TXEN_bm)
#define SETDDR  {VPORT3.DIR |= (1<<4)|(1<<5)|(1<<7); VPORT2.DIR |= 0x03; }
#define INIT()  { PORTCFG.VPCTRLB=PORTCFG_VP3MAP_PORTF_gc | PORTCFG_VP2MAP_PORTC_gc; CS_IDLE; RESET_IDLE; SETDDR; spi_init(); }

void spi_init(void)
{
   SPIF.CTRL=SPI_ENABLE_bm | SPI_MODE_3_gc | (1<<SPI_MASTER_bp) | (1<<SPI_CLK2X_bp);
}

#define write8(x)    {\
						 SPIF.DATA=x;\
                         while ((SPIF.STATUS & SPI_IF_bm)==0);\
                         SPIF.DATA;\
                     }
#define flush()   {\
                  }

#define PIN_LOW(p, b)        (p).OUT &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p).OUT |= (1<<(b))
#define PIN_OUTPUT(p, b)     (p).DIR |= (1<<(b))

#elif defined(__AVR_ATxmega32A4U__)     //3.49s @ 32MHz -O2
  #define CD_PORT VPORT2
  #define CD_PIN  1
  #define CS_PORT VPORT3
  #define CS_PIN  0
  #define RESET_PORT VPORT2
  #define RESET_PIN  0
  #define SD_PORT    PORTC
  #define SD_PIN     4
#define SPSRVAL (USART_CLK2X_bm | USART_RXEN_bm | USART_TXEN_bm)
#define SPCRVAL (USART_CMODE_MSPI_gc | 0x00 | 0x00)
#define SETDDR  {VPORT3.DIR |= (1<<0)|(1<<1)|(1<<3); VPORT2.DIR |= 0x03; PIN_HIGH(SD_PORT, SD_PIN); SD_PORT.DIR |= (1<<SD_PIN); }
#define INIT()  { PORTCFG.VPCTRLB=PORTCFG_VP13MAP_PORTD_gc | PORTCFG_VP02MAP_PORTC_gc; CS_IDLE; RESET_IDLE; SETDDR; spi_init(); }

void spi_init(void)
{
   USARTD0.CTRLB = SPSRVAL;
   USARTD0.CTRLC = SPCRVAL;   //mode #0 
//   PORTD.PIN1CTRL |= PORT_INVEN_bm;   //CPOL
   USARTD0.BAUDCTRLA = 0x00;     //F_CPU/2
   USARTD0.BAUDCTRLB = ((0x00 << USART_BSCALE_gp) & USART_BSCALE_gm) | 0x00;
   USARTD0.DATA; 
}

extern uint8_t running;

#define write8(x)    {\
						 while ((USARTD0.STATUS & USART_DREIF_bm) == 0) ;\
                         asm("cli");\
						 USARTD0.DATA = x;\
						 USARTD0.STATUS = USART_TXCIF_bm;\
						 asm("sei");\
						 running = 1;\
                     }
static inline uint8_t read8(void)    {
                         if (running) while ((USARTD0.STATUS & USART_RXCIF_bm) == 0) ;
						 return USARTD0.DATA;
                     }
#define flush()   {\
                         if (running) while ((USARTD0.STATUS & USART_TXCIF_bm) == 0) ;\
						 while ((USARTD0.STATUS & USART_RXCIF_bm) != 0) USARTD0.DATA;\
						 running = 0;\
                  }
static inline uint8_t xchg8(uint8_t x)    {
						 USARTD0.DATA = x;
                         while ((USARTD0.STATUS & USART_RXCIF_bm) == 0) ;
						 return USARTD0.DATA;
                     }
/*
#define write8(x)    {\
                         while ((USARTD0.STATUS & USART_DREIF_bm) == 0) ;\
                         USARTD0.DATA = x;\
                         while ((USARTD0.STATUS & USART_RXCIF_bm) == 0) ;\
                         USARTD0.DATA;\
                     }
#define flush()
*/

#define PIN_LOW(p, b)        (p).OUT &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p).OUT |= (1<<(b))
#define PIN_OUTPUT(p, b)     (p).DIR |= (1<<(b))

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168PB__)
  #define CD_PORT PORTB
  #define CD_PIN  PB1
  #define CS_PORT PORTB
  #define CS_PIN  PB2
  #define RESET_PORT PORTB
  #define RESET_PIN  PB0
  #define SD_PORT PORTD
  #define SD_PIN  PD4
#define SPCRVAL ((1<<SPE)|(1<<MSTR)|(0<<CPHA)|(0<<SPR0))
#define SETDDR  {DDRB |= (1<<5)|(1<<3)|(1<<2)|(1<<1)|(1<<0); DDRD |= (1<<4);}
#define INIT()  { CS_IDLE; RESET_IDLE; SETDDR; spi_init(); }
void spi_init(void)
{
   SPCR = SPCRVAL; 
   SPSR = (1<<SPI2X); 
   SPSR; 
   SPDR; 
}
#if 0
extern uint8_t running;
static inline uint8_t spi(uint8_t x)   { SPDR = x; while ((SPSR & 0x80) == 0); return running = SPDR; }
#define write8(x)                      spi(x)
#define xchg8(x)  spi(x)
#define read8()   running
#define flush()   
#elif 1
static inline void write8(uint8_t x)   { SPDR = x; while ((SPSR & 0x80) == 0); }
static inline uint8_t read8(void)      { while ((SPSR & 0x80) == 0); return SPDR; }
static inline uint8_t xchg8(uint8_t x) { write8(x); return read8(); }
static inline void flush(void)         { }
#else
extern uint8_t running;
static inline void write8(uint8_t x)    {
                         if (running) {
							 while ((SPSR & 0x80) == 0);
							 SPDR;
						 }
                         SPDR = x;
                         running = 1;
                     }
static inline uint8_t read8(void)    {
                         if (running) while ((SPSR & 0x80) == 0);
                         running = 0;
						 return SPDR;
                     }
static inline uint8_t xchg8(uint8_t x) { write8(x); return read8(); }
static inline void flush(void)   {
                      if (running) {
                          while ((SPSR & 0x80) == 0);
                      }
                      running = 0;
    				  SPDR;
                  }
#endif
#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))

#elif defined(__AVR_ATtiny1634__)
  #define CD_PORT PORTC
  #define CD_PIN  PC5
  #define CS_PORT PORTA
  #define CS_PIN  PA6
  #define RESET_PORT PORTA
  #define RESET_PIN  PA2
#define SETDDR     {DDRA |= (1<<6)|(1<<2); DDRB |= (1<<2); DDRC |= (1<<5)|(1<<1);}
#define INIT()     { CS_IDLE; RESET_IDLE; SETDDR; spi_init(); }
#define spi_init() { UCSR1B = (1<<TXEN1)|(1<<RXEN1); UCSR1C = (3<<UMSEL10); UBRR1 = 0; UDR1; }
#define write8(x)    {\
                         while ((UCSR1A & (1<<UDRE1)) == 0) ;\
                         UDR1 = x;\
                         if (running < 2) running++;\
                     }
#define flush()   {\
                      while (running--) {\
                         while ((UCSR1A & (1<<RXC1)) == 0) ;\
                         UDR1;\
                      }\
                      running = 0;\
                  }
#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))

#else
#error
#endif

#define TFTFAST spi_init()
#define SDFAST spi_init()
#define SDSLOW spi_init()
#define SDHI   PIN_HIGH(SD_PORT, SD_PIN)
#define SDLO   PIN_LOW(SD_PORT, SD_PIN)
#define spi_write(x)  write8(x)
#define spi_read()    read8()
#define spi_flush()   flush()
extern uint8_t running;

#endif /* TFTSPI_AVR_H_ */