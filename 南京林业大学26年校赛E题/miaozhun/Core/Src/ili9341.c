#include "ili9341.h"
extern SPI_HandleTypeDef hspi1;
static const uint8_t ili9341_init_seq[] = {
    0x01, 0x00,
    0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
    0xCF, 3, 0x00, 0xC1, 0x30,
    0xE8, 3, 0x85, 0x00, 0x78,
    0xEA, 2, 0x00, 0x00,
    0xED, 4, 0x64, 0x03, 0x12, 0x81,
    0xF7, 1, 0x20,
    0xC0, 1, 0x23,
    0xC1, 1, 0x10,
    0xC5, 2, 0x3E, 0x28,
    0xC7, 1, 0x86,
    0x36, 1, 0x48,
    0x3A, 1, 0x55,
    0xB1, 2, 0x00, 0x18,
    0xB6, 3, 0x08, 0x82, 0x27,
    0xF2, 1, 0x00,
    0x26, 1, 0x01,
    0xE0, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
    0xE1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
    0x11, 0x00,
    0x29, 0x00,
};

static void ILI9341_WriteCmd(uint8_t cmd)
{
    ILI9341_DC_LOW();
    ILI9341_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    ILI9341_CS_HIGH();
}

static void ILI9341_WriteData(uint8_t data)
{
    ILI9341_DC_HIGH();
    ILI9341_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
    ILI9341_CS_HIGH();
}

static void ILI9341_WriteData16(uint16_t data)
{
    uint8_t buf[2];
    buf[0] = data >> 8;
    buf[1] = data & 0xFF;
    ILI9341_DC_HIGH();
    ILI9341_CS_LOW();
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    ILI9341_CS_HIGH();
}

#if 0
static void ILI9341_WriteDataBulk(const uint8_t *data, uint16_t len)
{
    ILI9341_DC_HIGH();
    ILI9341_CS_LOW();
    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, HAL_MAX_DELAY);
    ILI9341_CS_HIGH();
}

static void ILI9341_WriteDataBulk16(const uint16_t *data, uint16_t len)
{
    ILI9341_DC_HIGH();
    ILI9341_CS_LOW();
    while (len--)
    {
        uint8_t buf[2];
        buf[0] = (*data) >> 8;
        buf[1] = (*data) & 0xFF;
        HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
        data++;
    }
    ILI9341_CS_HIGH();
}
#endif

void ILI9341_Init(void)
{
    ILI9341_RST_HIGH();
    ILI9341_CS_HIGH();
    HAL_Delay(50);

    ILI9341_RST_LOW();
    HAL_Delay(10);
    ILI9341_RST_HIGH();
    HAL_Delay(150);

    const uint8_t *p = ili9341_init_seq;
    while (p < ili9341_init_seq + sizeof(ili9341_init_seq))
    {
        uint8_t cmd = *p++;
        uint8_t len = *p++;
        if (len == 0)
        {
            ILI9341_WriteCmd(cmd);
        }
        else
        {
            ILI9341_DC_LOW();
            ILI9341_CS_LOW();
            HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
            ILI9341_DC_HIGH();
            HAL_SPI_Transmit(&hspi1, (uint8_t *)p, len, HAL_MAX_DELAY);
            ILI9341_CS_HIGH();
            p += len;
        }

        if (cmd == 0x01) HAL_Delay(10);
        else if (cmd == 0x11) HAL_Delay(150);
        else if (cmd == 0x29) HAL_Delay(50);
    }

    ILI9341_BL_ON();
    ILI9341_FillScreen(ILI9341_COLOR_BLACK);
}

void ILI9341_SetRotation(ILI9341_Rotation rotation)
{
    ILI9341_WriteCmd(0x36);
    ILI9341_WriteData(rotation);
}

void ILI9341_SetAddressWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    ILI9341_WriteCmd(0x2A);
    ILI9341_WriteData16(x1);
    ILI9341_WriteData16(x2);
    ILI9341_WriteCmd(0x2B);
    ILI9341_WriteData16(y1);
    ILI9341_WriteData16(y2);
    ILI9341_WriteCmd(0x2C);
}

void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= ILI9341_WIDTH || y >= ILI9341_HEIGHT) return;
    ILI9341_SetAddressWindow(x, y, x, y);
    ILI9341_WriteData16(color);
}

void ILI9341_FillScreen(uint16_t color)
{
    ILI9341_FillRect(0, 0, ILI9341_WIDTH, ILI9341_HEIGHT, color);
}

void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= ILI9341_WIDTH || y >= ILI9341_HEIGHT) return;
    if (x + w > ILI9341_WIDTH)  w = ILI9341_WIDTH - x;
    if (y + h > ILI9341_HEIGHT) h = ILI9341_HEIGHT - y;

    ILI9341_SetAddressWindow(x, y, x + w - 1, y + h - 1);

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    /* 使用较小的行缓冲区，分块传输 */
    uint8_t line[64];
    uint16_t buf_pixels = sizeof(line) / 2;
    uint16_t i;
    for (i = 0; i < buf_pixels && i < w; i++)
    {
        line[i * 2]     = hi;
        line[i * 2 + 1] = lo;
    }

    ILI9341_DC_HIGH();
    ILI9341_CS_LOW();
    while (h--)
    {
        uint16_t remaining = w;
        while (remaining > 0)
        {
            uint16_t chunk = (remaining > buf_pixels) ? buf_pixels : remaining;
            HAL_SPI_Transmit(&hspi1, line, chunk * 2, HAL_MAX_DELAY);
            remaining -= chunk;
        }
    }
    ILI9341_CS_HIGH();
}

void ILI9341_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ILI9341_DrawHLine(x, y, w, color);
    ILI9341_DrawHLine(x, y + h - 1, w, color);
    ILI9341_DrawVLine(x, y, h, color);
    ILI9341_DrawVLine(x + w - 1, y, h, color);
}

void ILI9341_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
    ILI9341_FillRect(x, y, w, 1, color);
}

void ILI9341_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color)
{
    ILI9341_FillRect(x, y, 1, h, color);
}

void ILI9341_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    /*
     * Standard all-octant Bresenham implementation.
     * The old code made dy negative only when y1 < y0, but still used
     * err = dx - dy.  For an upward LCD segment y was then never updated,
     * x ran past x1 forever, and Task4 froze on the first sine-wave rise.
     */
    int16_t dx = (x1 >= x0) ? (int16_t)(x1 - x0) : (int16_t)(x0 - x1);
    int16_t dy = (y1 >= y0) ? -(int16_t)(y1 - y0) : -(int16_t)(y0 - y1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx + dy;

    while (1)
    {
        ILI9341_DrawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = err * 2;
        if (e2 >= dy) { err += dy; x0 = (uint16_t)((int32_t)x0 + sx); }
        if (e2 <= dx) { err += dx; y0 = (uint16_t)((int32_t)y0 + sy); }
    }
}

void ILI9341_DrawCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t dx = 1;
    int16_t dy = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ILI9341_DrawPixel(cx, cy + r, color);
    ILI9341_DrawPixel(cx, cy - r, color);
    ILI9341_DrawPixel(cx + r, cy, color);
    ILI9341_DrawPixel(cx - r, cy, color);

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            dy += 2;
            f += dy;
        }
        x++;
        dx += 2;
        f += dx;
        ILI9341_DrawPixel(cx + x, cy + y, color);
        ILI9341_DrawPixel(cx - x, cy + y, color);
        ILI9341_DrawPixel(cx + x, cy - y, color);
        ILI9341_DrawPixel(cx - x, cy - y, color);
        ILI9341_DrawPixel(cx + y, cy + x, color);
        ILI9341_DrawPixel(cx - y, cy + x, color);
        ILI9341_DrawPixel(cx + y, cy - x, color);
        ILI9341_DrawPixel(cx - y, cy - x, color);
    }
}

void ILI9341_FillCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t dx = 1;
    int16_t dy = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ILI9341_DrawVLine(cx, cy - r, r * 2 + 1, color);

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            dy += 2;
            f += dy;
        }
        x++;
        dx += 2;
        f += dx;
        ILI9341_DrawVLine(cx + x, cy - y, y * 2 + 1, color);
        ILI9341_DrawVLine(cx - x, cy - y, y * 2 + 1, color);
        ILI9341_DrawVLine(cx + y, cy - x, x * 2 + 1, color);
        ILI9341_DrawVLine(cx - y, cy - x, x * 2 + 1, color);
    }
}

void ILI9341_SetBacklight(uint8_t on)
{
    if (on)
        ILI9341_BL_ON();
    else
        ILI9341_BL_OFF();
}

void ILI9341_WriteColor(uint16_t color)
{
    ILI9341_WriteData16(color);
}

static const uint8_t font5x7[96][5] = {
    {0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},
    {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},
    {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},
    {0x08,0x2A,0x1C,0x2A,0x08},
    {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},
    {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},
    {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},
    {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},
    {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},
    {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},
    {0x00,0x08,0x14,0x22,0x41},
    {0x14,0x14,0x14,0x14,0x14},
    {0x41,0x22,0x14,0x08,0x00},
    {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x01,0x01},
    {0x3E,0x41,0x41,0x51,0x32},
    {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x04,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},
    {0x7F,0x20,0x18,0x20,0x7F},
    {0x63,0x14,0x08,0x14,0x63},
    {0x03,0x04,0x78,0x04,0x03},
    {0x61,0x51,0x49,0x45,0x43},
    {0x00,0x00,0x7F,0x41,0x41},
    {0x02,0x04,0x08,0x10,0x20},
    {0x41,0x41,0x7F,0x00,0x00},
    {0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00},
    {0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},
    {0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F},
    {0x38,0x54,0x54,0x54,0x18},
    {0x08,0x7E,0x09,0x01,0x02},
    {0x08,0x14,0x54,0x54,0x3C},
    {0x7F,0x08,0x04,0x04,0x78},
    {0x00,0x44,0x7D,0x40,0x00},
    {0x20,0x40,0x44,0x3D,0x00},
    {0x00,0x7F,0x10,0x28,0x44},
    {0x00,0x41,0x7F,0x40,0x00},
    {0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},
    {0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7C},
    {0x7C,0x08,0x04,0x04,0x08},
    {0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},
    {0x3C,0x40,0x40,0x20,0x7C},
    {0x1C,0x20,0x40,0x20,0x1C},
    {0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44},
    {0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},
    {0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00},
    {0x08,0x08,0x2A,0x1C,0x08},
    {0x08,0x1C,0x2A,0x08,0x08},
};

void ILI9341_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size)
{
    if (c < ' ' || c > '~') c = ' ';
    if (size == 0 || size > 5) size = 1;
    uint8_t idx = c - ' ';

    ILI9341_SetAddressWindow(x, y, x + 6 * size - 1, y + 8 * size - 1);

    uint8_t line_buf[60];
    ILI9341_DC_HIGH();
    ILI9341_CS_LOW();

    for (uint8_t row = 0; row < 8; row++)
    {
        for (uint8_t sy = 0; sy < size; sy++)
        {
            for (uint8_t col = 0; col < 5; col++)
            {
                uint8_t colByte = font5x7[idx][col];
                uint16_t c16 = (colByte & (1 << row)) ? color : bg;
                uint8_t hi = c16 >> 8;
                uint8_t lo = c16 & 0xFF;
                for (uint8_t sx = 0; sx < size; sx++)
                {
                    line_buf[(col * size + sx) * 2]     = hi;
                    line_buf[(col * size + sx) * 2 + 1] = lo;
                }
            }
            for (uint8_t sx = 0; sx < size; sx++)
            {
                line_buf[(5 * size + sx) * 2]     = bg >> 8;
                line_buf[(5 * size + sx) * 2 + 1] = bg & 0xFF;
            }
            HAL_SPI_Transmit(&hspi1, line_buf, 6 * size * 2, HAL_MAX_DELAY);
        }
    }
    ILI9341_CS_HIGH();
}

void ILI9341_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t size)
{
    while (*str)
    {
        if (x + 6 * size > ILI9341_WIDTH) { x = 0; y += 8 * size; }
        if (*str == '\n') { x = 0; y += 8 * size; str++; continue; }
        ILI9341_DrawChar(x, y, *str, color, bg, size);
        x += 6 * size;
        str++;
    }
}
