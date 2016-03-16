#include "ST7735_kbv.h"
#include "serial_kbv.h"

ST7735_kbv::ST7735_kbv():Adafruit_GFX(128, 160)
{
    INIT();
    CS_IDLE;
    RESET_IDLE;
}

void ST7735_kbv::reset(void)
{
    wait_ms(50);
    RESET_ACTIVE;
    wait_ms(100);
    RESET_IDLE;
    wait_ms(100);
}

void ST7735_kbv::WriteCmdData(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}

static uint32_t readRegister(uint8_t reg)
{
    uint32_t ret;
	uint8_t bits = 8;
	if (reg == 4) bits = 25;
	if (reg == 9) bits = 33;
    CS_ACTIVE;
    WriteCmd(reg);
    CD_DATA;                    //
	SDIO_INMODE();
    ret = readbits(bits);
    CS_IDLE;
	SDIO_OUTMODE();
    return ret;	
}

uint16_t ST7735_kbv::readReg(uint16_t reg)
{
    return readRegister(reg);
}

uint32_t ST7735_kbv::readReg32(uint16_t reg)
{
	return readRegister(reg);
}

uint16_t ST7735_kbv::readID(void)
{
    return readRegister(4) >> 8;	
}

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

int16_t ST7735_kbv::readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
	uint8_t r, g, b;
	int16_t n = w * h;    // we are NEVER going to read > 32k pixels at once
	setAddrWindow(x, y, x + w - 1, y + h - 1);
	CS_ACTIVE;
	WriteCmd(ST7735_RAMRD);
	CD_DATA;
	SDIO_INMODE();        // do this while CS is Active

	r = readbits(9);	  // needs 1 dummy read (8) for ILI9163
	while (n-- > 0) {
		r = readbits(8);
		g = readbits(8);
		b = readbits(8);
		*block++ = color565(r, g, b);
	}
	CS_IDLE;
	SDIO_OUTMODE();      //do this when CS is Idle
    setAddrWindow(0, 0, width() - 1, height() - 1);
    return 0;
}

void ST7735_kbv::setRotation(uint8_t r)
{
    uint16_t mac = 0x800;
    Adafruit_GFX::setRotation(r & 3);
    switch (rotation) {
    case 0:
        mac = 0xD800;
        break;
    case 1:        //LANDSCAPE 90 degrees
        mac = 0xB800;
        break;
    case 2:
        mac = 0x0800;
        break;
    case 3:
        mac = 0x6800;
        break;
    }
    WriteCmdData(ST7735_MADCTL, mac & ~0x0800);
}

void ST7735_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // ILI934X just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
    CS_ACTIVE;
    WriteCmd(ST7735_CASET);
    spibuf[0] = x >> 8;
    spibuf[1] = x;
    CD_DATA;
    write8_block(spibuf, 2);
    WriteCmd(ST7735_RASET);
    spibuf[0] = y >> 8;
    spibuf[1] = y;
    CD_DATA;
    write8_block(spibuf, 2);
    WriteCmd(ST7735_RAMWR);
    spibuf[0] = color >> 8;
    spibuf[1] = color;
    CD_DATA;
    write8_block(spibuf, 2);
    CS_IDLE;
}

void ST7735_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
    CS_ACTIVE;
    WriteCmd(ST7735_CASET);
    spibuf[0] = x >> 8;
    spibuf[1] = x;
    spibuf[2] = x1 >> 8;
    spibuf[3] = x1;
    CD_DATA;
    write8_block(spibuf, 4);
    WriteCmd(ST7735_RASET);
    spibuf[0] = y >> 8;
    spibuf[1] = y;
    spibuf[2] = y1 >> 8;
    spibuf[3] = y1;
    CD_DATA;
    write8_block(spibuf, 4);
    CS_IDLE;
}

void ST7735_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int16_t end;
    if (w < 0) {
        w = -w;
        x -= w;
    }                           //+ve w
    end = x + w;
    if (x < 0)
        x = 0;
    if (end > width())
        end = width();
    w = end - x;
    if (h < 0) {
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
        y = 0;
    if (end > height())
        end = height();
    h = end - y;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(ST7735_RAMWR);
    CD_DATA;
    if (h > w) {
        end = h;
        h = w;
        w = end;
    }
    while (h-- > 0) {
        write16_N(color, w);
    }
    CS_IDLE;
    setAddrWindow(0, 0, width() - 1, height() - 1);
}

void ST7735_kbv::pushColors(uint16_t * block, int16_t n, bool first)
{
    uint16_t color;
    CS_ACTIVE;
    if (first) {
        WriteCmd(ST7735_RAMWR);
    }
    CD_DATA;
    while (n-- > 0) {
        color = *block++;
        write16(color);
    }
    CS_IDLE;
}

void ST7735_kbv::pushColors(const uint8_t * block, int16_t n, bool first)
{
    uint16_t color;
	uint8_t h, l;
	CS_ACTIVE;
    if (first) {
        WriteCmd(ST7735_RAMWR);
    }
    CD_DATA;
    while (n-- > 0) {
        l = pgm_read_byte(block++);
        h = pgm_read_byte(block++);
        color = h<<8 | l;
		write16(color);
    }
    CS_IDLE;
}

void ST7735_kbv::invertDisplay(boolean i)
{
    WriteCmdData(i ? ST7735_INVON : ST7735_INVOFF, 0);
}

void ST7735_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
    int16_t vsp;
    vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    CS_ACTIVE;
    WriteCmd(0x0033);
    WriteData(top);             //TOP
    write16(scrollines);
    write16(bfa);

    WriteCmdData(0x0037, vsp);       //VL#

}

#define TFTLCD_DELAY 0xFF

const uint8_t PROGMEM table7735R[] = {
    //  (COMMAND_BYTE), n, data_bytes....
    (ST7735_SWRESET), 0,        // software reset
    TFTLCD_DELAY, 50,
    (ST7735_SLPOUT), 0,         //Sleep exit
    TFTLCD_DELAY, 250,
    //ST7735R Frame Rate
    (ST7735_FRMCTR1), 3, 0x01, 0x2C, 0x2D,
    (ST7735_FRMCTR2), 3, 0x01, 0x2C, 0x2D,
    (ST7735_FRMCTR3), 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D,
    (ST7735_INVCTR), 1, 0x07,   //Column inversion
    //ST7735R Power Sequence
    (ST7735_PWCTR1), 3, 0xA2, 0x02, 0x84,
    (ST7735_PWCTR2), 1, 0xC5,
    (ST7735_PWCTR3), 2, 0x0A, 0x00,
    (ST7735_PWCTR4), 2, 0x8A, 0x2A,
    (ST7735_PWCTR5), 2, 0x8A, 0xEE,
    (ST7735_VMCTR1), 1, 0x0E,   //VCOM
    (ST7735_INVOFF), 0, //no inversion
    //(ST7735_MADCTL), 1, 0xC8, //MX, MY, RGB mode
    (ST7735_MADCTL), 1, 0x00,   //MX, MY, RGB mode
    //ST7735R Gamma Sequence
    (ST7735_GMCTRP1), 16,
    0x0f, 0x1a, 0x0f, 0x18, 0x2f, 0x28, 0x20, 0x22, 0x1f, 0x1b, 0x23, 0x37,
    0x00, 0x07, 0x02, 0x10,
    (ST7735_GMCTRN1), 16,
    0x0f, 0x1b, 0x0f, 0x17, 0x33, 0x2c, 0x29, 0x2e, 0x30, 0x30, 0x39, 0x3f,
    0x00, 0x07, 0x03, 0x10,
    (ST7735_CASET), 4, 0x00, 0x00, 0x00, 0x7f,
    (ST7735_RASET), 4, 0x00, 0x00, 0x00, 0x9f,
    (0xF0), 1, 0x01,            //Enable test command
    (0xF6), 1, 0x00,            //Disable ram power save mode
    (ST7735_COLMOD), 1, 0x05,   //65k mode
    (ST7735_DISPON), 0,         //Display on
};

void ST7735_kbv::begin(uint16_t ID)
{
    _lcd_ID = ID;
    uint8_t *p = (uint8_t *) table7735R;
    int16_t size = sizeof(table7735R);
    reset();
    while (size > 0) {
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY) {
            delay(len);
            len = 0;
        } else {
            CS_ACTIVE;
            WriteCmd(cmd);
            CD_DATA;
            for (uint8_t d = 0; d < len; d++) {
                uint8_t x = pgm_read_byte(p++);
                xchg8(x);
            }
            CS_IDLE;
        }
        size -= len + 2;
    }
    setRotation(0);             //PORTRAIT
}
