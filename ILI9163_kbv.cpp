#define __OFFSET 32      //for Red 128x128 module

#include "ILI9163_kbv.h"
#include "serial_kbv.h"

ILI9163_kbv::ILI9163_kbv():Adafruit_GFX(128, 128)
{
    INIT();
    CS_IDLE;
    RESET_IDLE;
}

void ILI9163_kbv::reset(void)
{
    wait_ms(50);
    RESET_ACTIVE;
    wait_ms(100);
    RESET_IDLE;
    wait_ms(100);
}

void ILI9163_kbv::WriteCmdData(uint16_t cmd, uint16_t dat)
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
    CD_DATA;                    //should do a flush()
	SDIO_INMODE();        // do this while CS is Active
    ret = readbits(bits);
    CS_IDLE;
	SDIO_OUTMODE();      //do this when CS is Idle
    return ret;	
}

uint16_t ILI9163_kbv::readReg(uint16_t reg)
{
    return readRegister(reg);
}

uint32_t ILI9163_kbv::readReg32(uint16_t reg)
{
	return readRegister(reg);
}

uint16_t ILI9163_kbv::readID(void)
{
    return readRegister(4) >> 8;	
}

#define ILI9163_NOP     0x00
#define ILI9163_SWRESET 0x01
#define ILI9163_RDDID   0x04
#define ILI9163_RDDST   0x09

#define ILI9163_SLPIN   0x10
#define ILI9163_SLPOUT  0x11
#define ILI9163_PTLON   0x12
#define ILI9163_NORON   0x13

#define ILI9163_INVOFF  0x20
#define CMD_GAMMASET 	0x26//Gamma Set (0x01[1],0x02[2],0x04[3],0x08[4])
#define ILI9163_INVON   0x21
#define ILI9163_DISPOFF 0x28
#define ILI9163_DISPON  0x29
#define ILI9163_CASET   0x2A
#define ILI9163_RASET   0x2B
#define ILI9163_RAMWR   0x2C
#define ILI9163_RAMRD   0x2E

#define ILI9163_PTLAR   0x30
#define CMD_VSCLLDEF	0x33//Vertical Scroll Definition
#define ILI9163_COLMOD  0x3A
#define ILI9163_MADCTL  0x36
#define CMD_VSSTADRS	0x37//Vertical Scrolling Start address

#define ILI9163_FRMCTR1 0xB1
#define ILI9163_FRMCTR2 0xB2
#define ILI9163_FRMCTR3 0xB3
#define ILI9163_INVCTR  0xB4
#define CMD_DFUNCTR 	0xB6//Display Fuction set 5
#define CMD_SDRVDIR 	0xB7//Source Driver Direction Control
#define CMD_GDRVDIR 	0xB8//Gate Driver Direction Control

#define ILI9163_PWCTR1  0xC0
#define ILI9163_PWCTR2  0xC1
#define ILI9163_PWCTR3  0xC2
#define ILI9163_PWCTR4  0xC3
#define ILI9163_PWCTR5  0xC4
#define ILI9163_VMCTR1  0xC5
#define CMD_VCOMCTR2  	0xC6//VCOM_Control 2
#define CMD_VCOMOFFS  	0xC7//VCOM Offset Control

#define ILI9163_RDID1   0xDA
#define ILI9163_RDID2   0xDB
#define ILI9163_RDID3   0xDC
#define ILI9163_RDID4   0xDD

//#define ILI9163_PWCTR6  0xFC

#define ILI9163_GMCTRP1 0xE0
#define ILI9163_GMCTRN1 0xE1
#define CMD_GAMRSEL		0xF2//GAM_R_SEL

int16_t ILI9163_kbv::readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{

	uint8_t r, g, b;
	int16_t n = w * h;    // we are NEVER going to read > 32k pixels at once
	setAddrWindow(x, y, x + w - 1, y + h - 1);
	CS_ACTIVE;
	WriteCmd(ILI9163_RAMRD);
	CD_DATA;
	SDIO_INMODE();        // do this while CS is Active

	// needs 1 dummy read
	r = readbits(8);    //ST7735 needs 9
	while (n-- > 0) {

		r = readbits(8);
		g = readbits(8);
		b = readbits(8);
		*block++ = color565(r, g, b);
	}
	CS_IDLE;
	SDIO_OUTMODE();      //do this when CS is Idle

    return 0;
}

void ILI9163_kbv::setRotation(uint8_t r)
{
    uint16_t mac = 0x800;
    Adafruit_GFX::setRotation(r & 3);
    switch (rotation) {
    case 0:
        mac = 0x0800;
        break;
    case 1:
        mac = 0xB800;
        break;
    case 2:
        mac = 0xD800;
        break;
    case 3:
        mac = 0x6800;
        break;
    }
    WriteCmdData(ILI9163_MADCTL, mac);
	setAddrWindow(0, 0, width() - 1, height() - 1);
	vertScroll(0, height(), 0);
}

void ILI9163_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // ILI934X just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
	if (rotation == 0) y += __OFFSET;
	if (rotation == 3) x += __OFFSET;
    CS_ACTIVE;
    WriteCmd(ILI9163_CASET);
    spibuf[0] = x >> 8;
    spibuf[1] = x;
    CD_DATA;
    write8_block(spibuf, 2);
    WriteCmd(ILI9163_RASET);
    spibuf[0] = y >> 8;
    spibuf[1] = y;
    CD_DATA;
    write8_block(spibuf, 2);
    WriteCmd(ILI9163_RAMWR);
    spibuf[0] = color >> 8;
    spibuf[1] = color;
    CD_DATA;
    write8_block(spibuf, 2);
    CS_IDLE;
}

void ILI9163_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
	if (rotation == 0) y += __OFFSET, y1 += __OFFSET;
	if (rotation == 3) x += __OFFSET, x1 += __OFFSET;
    CS_ACTIVE;
    WriteCmd(ILI9163_CASET);
    spibuf[0] = x >> 8;
    spibuf[1] = x;
    spibuf[2] = x1 >> 8;
    spibuf[3] = x1;
    CD_DATA;
    write8_block(spibuf, 4);
    WriteCmd(ILI9163_RASET);
    spibuf[0] = y >> 8;
    spibuf[1] = y;
    spibuf[2] = y1 >> 8;
    spibuf[3] = y1;
    CD_DATA;
    write8_block(spibuf, 4);
    CS_IDLE;
}

void ILI9163_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
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
    WriteCmd(ILI9163_RAMWR);
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

void ILI9163_kbv::pushColors(uint16_t * block, int16_t n, bool first)
{
    uint16_t color;
    CS_ACTIVE;
    if (first) {
        WriteCmd(ILI9163_RAMWR);
    }
    CD_DATA;
    while (n-- > 0) {
        color = *block++;
        write16(color);
    }
    CS_IDLE;
}

void ILI9163_kbv::pushColors(const uint8_t * block, int16_t n, bool first)
{
    uint16_t color;
	uint8_t h, l;
	CS_ACTIVE;
    if (first) {
        WriteCmd(ILI9163_RAMWR);
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

void ILI9163_kbv::invertDisplay(boolean i)
{
    WriteCmdData(i ? ILI9163_INVON : ILI9163_INVOFF, 0);
}

void ILI9163_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    if (rotation == 0 || rotation == 3) top += __OFFSET;
    int16_t vsp = top + offset;      // vertical start position
	if (vsp < 0)
        vsp += HEIGHT;       //keep in unsigned range
    CS_ACTIVE;
    WriteCmd(0x0033);
    WriteData(top);             //TOP
    write16(scrollines);
    write16(HEIGHT + __OFFSET - top - scrollines);

    WriteCmdData(0x0037, vsp);       //VL#

}

#define TFTLCD_DELAY 0xFF

const uint8_t PROGMEM table9163C[] = {
    //  (COMMAND_BYTE), n, data_bytes....
    (ILI9163_SWRESET), 0,        // software reset
    TFTLCD_DELAY, 250,
    TFTLCD_DELAY, 250,
    (ILI9163_SLPOUT), 0,         //Sleep exit
    TFTLCD_DELAY, 5,
    (ILI9163_COLMOD), 1, 0x05,   //65k mode
    TFTLCD_DELAY, 5,
    (CMD_GAMMASET), 1, 0x04,
    TFTLCD_DELAY, 1,
    (CMD_GAMRSEL), 1, 0x01,
    TFTLCD_DELAY, 1,
    (ILI9163_NORON), 0,         //Normal
    (CMD_DFUNCTR), 2, 0xFF, 0x06, //Display Function set 5 ??NL
    (CMD_SDRVDIR), 1, 0x00,
    (CMD_GDRVDIR), 1, 0x00,
	(ILI9163_GMCTRP1), 16,
    0x0f, 0x1a, 0x0f, 0x18, 0x2f, 0x28, 0x20, 0x22, 0x1f, 0x1b, 0x23, 0x37,
    0x00, 0x07, 0x02, 0x10,
    (ILI9163_GMCTRN1), 16,
    0x0f, 0x1b, 0x0f, 0x17, 0x33, 0x2c, 0x29, 0x2e, 0x30, 0x30, 0x39, 0x3f,
    0x00, 0x07, 0x03, 0x10,
    (ILI9163_FRMCTR1), 2, 0x08, 0x02,
    (ILI9163_INVCTR), 1, 0x07,   //Column inversion
    TFTLCD_DELAY, 1,
    (ILI9163_PWCTR1), 2, 0x0A, 0x02,
    TFTLCD_DELAY, 1,
    (ILI9163_PWCTR2), 1, 0x02,
    TFTLCD_DELAY, 1,
    (ILI9163_VMCTR1), 2, 0x50, 0x63,   //VCOM (#99)
    TFTLCD_DELAY, 1,
    (CMD_VCOMOFFS), 1, 0,
    TFTLCD_DELAY, 1,
	(CMD_VSCLLDEF), 6, 0, __OFFSET, 0, 128 + __OFFSET, 0, 0,
    (ILI9163_MADCTL), 1, 0x00,   //MX, MY, RGB mode
    (ILI9163_PTLAR), 4, 0x00, 0x00, 0x00, 0x7f,
    (ILI9163_CASET), 4, 0x00, 0x00, 0x00, 0x7f,
    (ILI9163_RASET), 4, 0x00, 0x00, 0x00, 0x7f,
//    (0xF0), 1, 0x01,            //Enable test command
//    (0xF6), 1, 0x00,            //Disable ram power save mode
    (ILI9163_DISPON), 0,         //Display on
};

void ILI9163_kbv::begin(uint16_t ID)
{
    _lcd_ID = ID;
    uint8_t *p = (uint8_t *) table9163C;
    int16_t size = sizeof(table9163C);
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
