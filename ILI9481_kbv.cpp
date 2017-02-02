#include "ILI9481_kbv.h"
#define NINEBITS
//#include "serial9_kbv.h"
//#include "serial_kbv.h"
#include "serial_stm32_kbv.h"

ILI9481_kbv::ILI9481_kbv():Adafruit_GFX(320, 480)
{
}

static uint8_t done_reset;

void ILI9481_kbv::reset(void)
{
    done_reset = 1;
    INIT();
    CS_IDLE;
    RESET_IDLE;
	wait_ms(50);
	RESET_ACTIVE;
	wait_ms(100);
	RESET_IDLE;
	wait_ms(100);
}

void ILI9481_kbv::pushCommand(uint16_t cmd, uint8_t * block, int8_t N)
{
    CS_ACTIVE;
    WriteCmd(cmd);
	write8_block(block, N);
    CS_IDLE;
}

#define ILI9481_CMD_NOP                             0x00
#define ILI9481_CMD_SOFTWARE_RESET                  0x01
#define ILI9481_CMD_READ_RED                        0x06
#define ILI9481_CMD_READ_GREEN                      0x07
#define ILI9481_CMD_READ_BLUE                       0x08
#define ILI9481_CMD_READ_DISP_POWER                 0x0A
#define ILI9481_CMD_READ_DISP_MADCTRL               0x0B
#define ILI9481_CMD_READ_DISP_PIXEL_FORMAT          0x0C
#define ILI9481_CMD_READ_DISP_IMAGE_FORMAT          0x0D
#define ILI9481_CMD_READ_DISP_SIGNAL_MODE           0x0E
#define ILI9481_CMD_READ_DISP_SELF_DIAGNOSTIC       0x0F
#define ILI9481_CMD_ENTER_SLEEP_MODE                0x10
#define ILI9481_CMD_SLEEP_OUT                       0x11
#define ILI9481_CMD_PARTIAL_MODE_ON                 0x12
#define ILI9481_CMD_NORMAL_DISP_MODE_ON             0x13
#define ILI9481_CMD_DISP_INVERSION_OFF              0x20
#define ILI9481_CMD_DISP_INVERSION_ON               0x21
#define ILI9481_CMD_GAMMA_SET                       0x26
#define ILI9481_CMD_DISPLAY_OFF                     0x28
#define ILI9481_CMD_DISPLAY_ON                      0x29
#define ILI9481_CMD_COLUMN_ADDRESS_SET              0x2A
#define ILI9481_CMD_PAGE_ADDRESS_SET                0x2B
#define ILI9481_CMD_MEMORY_WRITE                    0x2C
#define ILI9481_CMD_COLOR_SET                       0x2D
#define ILI9481_CMD_MEMORY_READ                     0x2E
#define ILI9481_CMD_PARTIAL_AREA                    0x30
#define ILI9481_CMD_VERT_SCROLL_DEFINITION          0x33
#define ILI9481_CMD_TEARING_EFFECT_LINE_OFF         0x34
#define ILI9481_CMD_TEARING_EFFECT_LINE_ON          0x35
#define ILI9481_CMD_MEMORY_ACCESS_CONTROL           0x36
#define ILI9481_CMD_VERT_SCROLL_START_ADDRESS       0x37
#define ILI9481_CMD_IDLE_MODE_OFF                   0x38
#define ILI9481_CMD_IDLE_MODE_ON                    0x39
#define ILI9481_CMD_COLMOD_PIXEL_FORMAT_SET         0x3A
#define ILI9481_CMD_WRITE_MEMORY_CONTINUE           0x3C
#define ILI9481_CMD_READ_MEMORY_CONTINUE            0x3E
#define ILI9481_CMD_SET_TEAR_SCANLINE               0x44
#define ILI9481_CMD_GET_SCANLINE                    0x45
#define ILI9481_CMD_COMMAND_ACCESS                  0xB0
#define ILI9481_CMD_READ_ID                         0xBF

uint16_t ILI9481_kbv::readReg(uint16_t reg)
{
    uint8_t h, l;
    if (!done_reset) reset();
    CS_ACTIVE;
    WriteCmd(reg);
    CD_DATA;                    //should do a flush()

    // needs 1 dummy read
    //     h = xchg8(0);    
    h = xchg8(0xFF);
    l = xchg8(0xFF);
    CS_IDLE;
    return (h << 8) | l;
}

uint32_t ILI9481_kbv::readReg32(uint16_t reg)
{
	uint32_t ret = 0;
	CS_ACTIVE;
	WriteCmd(reg);
	CD_DATA;                    //should do a flush()
    
	readbits(1);
    for (uint8_t cnt = 5; cnt--; ) {
		ret <<= 8;
		ret |= xchg8(0);
	}
	CS_IDLE;
	return ret;
}

int16_t ILI9481_kbv::readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
    uint8_t r, g, b;
	  int16_t n = w * h;    // we are NEVER going to read > 32k pixels at once
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(ILI9481_CMD_MEMORY_READ);
    CD_DATA;

    // needs 1 dummy read
//    r = xchg8(0xFF);
    r = readbits(9);
    while (n-- > 0) {
        b = xchg8(0xFF);
        g = xchg8(0xFF);
        r = xchg8(0xFF);
		*block++ = color565(r, g, b);
    }
    CS_IDLE;
    setAddrWindow(0, 0, width() - 1, height() - 1);
    return 0;
}

void ILI9481_kbv::setRotation(uint8_t r)
{
    uint8_t mac = 0x00;
    Adafruit_GFX::setRotation(r & 3);
    switch (rotation) {
    case 0:
        mac = 0x08;
        break;
    case 1:        //LANDSCAPE 90 degrees 
        mac = 0x29;
        break;
    case 2:
        mac = 0x0B;
        break;
    case 3:
        mac = 0x2A;
        break;
    }
	mac ^= (_lcd_xor);
    pushCommand(ILI9481_CMD_MEMORY_ACCESS_CONTROL, &mac, 1);
}

void ILI9481_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // ILI934X just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
    setAddrWindow(x, y, x, y);
#if 1
    CS_ACTIVE;
	WriteCmd(ILI9481_CMD_MEMORY_WRITE);
    CD_DATA;
	write24_N(color, 1);
    CS_IDLE;
#else
	spibuf[0] = (color >> 8);
    spibuf[1] = (color >> 3);
    spibuf[2] = (color << 3);
	pushCommand(ILI9481_CMD_MEMORY_WRITE, spibuf, 3);
#endif
}

void ILI9481_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
    spibuf[0] = x >> 8;
    spibuf[1] = x;
    spibuf[2] = x1 >> 8;
    spibuf[3] = x1;
	pushCommand(ILI9481_CMD_COLUMN_ADDRESS_SET, spibuf, 4);
    spibuf[0] = y >> 8;
    spibuf[1] = y;
    spibuf[2] = y1 >> 8;
    spibuf[3] = y1;
	pushCommand(ILI9481_CMD_PAGE_ADDRESS_SET, spibuf, 4);
}

void ILI9481_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
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
    WriteCmd(ILI9481_CMD_MEMORY_WRITE);
    CD_DATA;
	if (h > w) { end = h; h = w; w = end; } 
    while (h-- > 0) {
        write24_N(color, w);
    }
    CS_IDLE;
    setAddrWindow(0, 0, width() - 1, height() - 1);
}

void ILI9481_kbv::pushColors(uint16_t * block, int16_t n, bool first)
{
    uint16_t color;
    CS_ACTIVE;
    if (first) {
        WriteCmd(ILI9481_CMD_MEMORY_WRITE);
    }
    CD_DATA;
    while (n-- > 0) {
        color = *block++;
        write24(color);
    }
    CS_IDLE;
}

void ILI9481_kbv::pushColors(const uint8_t * block, int16_t n, bool first)
{
    uint16_t color;
	uint8_t h, l;
	CS_ACTIVE;
    if (first) {
        WriteCmd(ILI9481_CMD_MEMORY_WRITE);
    }
    CD_DATA;
    while (n-- > 0) {
        l = pgm_read_byte(block++);
        h = pgm_read_byte(block++);
        color = h<<8 | l;
		write24(color);
    }
    CS_IDLE;
}

void ILI9481_kbv::invertDisplay(boolean i)
{
    pushCommand(i ? ILI9481_CMD_DISP_INVERSION_ON : ILI9481_CMD_DISP_INVERSION_OFF, NULL, 0);
}
    
void ILI9481_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
    int16_t vsp;
    vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    spibuf[0] = top>>8;
	spibuf[1] = top;
	spibuf[2] = scrollines>>8;
	spibuf[3] = scrollines;
	spibuf[4] = bfa>>8;
	spibuf[5] = bfa;
	pushCommand(0x33, spibuf, 6);
    spibuf[0] = vsp>>8;
	spibuf[1] = vsp;
	pushCommand(0x37, spibuf, 2);
}

#define TFTLCD_DELAY8 0xFF
        static const uint8_t ILI9481_RGB_regValues[] PROGMEM = {    // 320x480
            0x01, 0,            //Soft Reset
            TFTLCD_DELAY8, 125,
            0x11, 0,            //Sleep Out
            TFTLCD_DELAY8, 20,
            0xB0, 1, 0x00,
            0xD0, 3, 0x07, 0x41, 0x1D,  // SETPOWER [00 43 18]
            0xD1, 3, 0x00, 0x2B, 0x1F,  // SETVCOM  [00 00 00] x0.900, x1.32
            0xD2, 2, 0x01, 0x11,        // SETNORPOW for Normal Mode [01 22]
            0xC0, 6, 0x00, 0x3B, 0x00, 0x02, 0x11, 0x00,     //SETPANEL [10 3B 00 02 11]
            0xC5, 1, 0x03,      //SETOSC Frame Rate [03]
            0xC6, 1, 0x80,      //SETRGB interface control
			0xC8, 12, 0x00, 0x14, 0x33, 0x10, 0x00, 0x16, 0x44, 0x36, 0x77, 0x00, 0x0F, 0x00,
            0xF3, 2, 0x40, 0x0A,			
            0xF0, 1, 0x08,
            0xF6, 1, 0x84,
            0xF7, 1, 0x80,
            0x0C, 2, 0x00, 0x55, //RDCOLMOD
			0xB4, 1, 0x00,      //SETDISPLAY
//			0xB3, 4, 0x00, 0x01, 0x06, 0x01,  //SETGRAM simple example
			0xB3, 4, 0x00, 0x01, 0x06, 0x30,  //jpegs example
            0x36, 1, 0x48,      //Memory Access [00]
            0x3A, 1, 0x66,      //Interlace Pixel Format [XX]
//			0x20, 0,            //INVOFF
//			0x21, 0,            //INVON

//            0x3A, 1, 0x55,      //Interlace Pixel Format [XX]
            0x11, 0,            //Sleep Out
            TFTLCD_DELAY8, 120,
            0x29, 0,            //Display On
            TFTLCD_DELAY8, 25,
        };

void ILI9481_kbv::begin(uint16_t ID)
{
    _lcd_ID = ID;
	_lcd_xor = 0x02;
    uint8_t *p = (uint8_t *)ILI9481_RGB_regValues;
    int16_t size = sizeof(ILI9481_RGB_regValues);
    reset();
    while (size > 0) {
	    uint8_t cmd = pgm_read_byte(p++);
	    uint8_t len = pgm_read_byte(p++);
	    if (cmd == TFTLCD_DELAY8) {
		    delay(len);
		    len = 0;
		} else {
		    CS_ACTIVE;
		    WriteCmd(cmd);
		    CD_DATA;
		    for (uint8_t d = 0; d < len; d++) {
			    uint8_t x = pgm_read_byte(p++);
			    WriteDat8(x);
		    }
		    CS_IDLE;
	    }
	    size -= len + 2;
    }
    setRotation(0);             //PORTRAIT
}
