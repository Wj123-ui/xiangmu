#ifndef __ILI9341_H
#define __ILI9341_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define ILI9341_WIDTH   240
#define ILI9341_HEIGHT  320

#define ILI9341_CS_PIN      GPIO_PIN_4
#define ILI9341_CS_PORT     GPIOA
#define ILI9341_DC_PIN      GPIO_PIN_4
#define ILI9341_DC_PORT     GPIOC
#define ILI9341_RST_PIN     GPIO_PIN_5


#define ILI9341_RST_PORT    GPIOC

#define ILI9341_CS_LOW()    HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_RESET)
#define ILI9341_CS_HIGH()   HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_SET)
#define ILI9341_DC_LOW()    HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_RESET)
#define ILI9341_DC_HIGH()   HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_SET)
#define ILI9341_RST_LOW()   HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_RESET)
#define ILI9341_RST_HIGH()  HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_SET)
#define ILI9341_BL_ON()     ((void)0)
#define ILI9341_BL_OFF()    ((void)0)

#define ILI9341_MADCTL_MY  0x80
#define ILI9341_MADCTL_MX  0x40
#define ILI9341_MADCTL_MV  0x20
#define ILI9341_MADCTL_ML  0x10
#define ILI9341_MADCTL_RGB 0x00
#define ILI9341_MADCTL_BGR 0x08
#define ILI9341_MADCTL_MH  0x04

#define ILI9341_COLOR_BLACK       0x0000
#define ILI9341_COLOR_WHITE       0xFFFF
#define ILI9341_COLOR_RED         0xF800
#define ILI9341_COLOR_GREEN       0x07E0
#define ILI9341_COLOR_BLUE        0x001F
#define ILI9341_COLOR_CYAN        0x07FF
#define ILI9341_COLOR_MAGENTA     0xF81F
#define ILI9341_COLOR_YELLOW      0xFFE0
#define ILI9341_COLOR_ORANGE      0xFD20
#define ILI9341_COLOR_GRAY        0x8410

#define RGB565(r, g, b)  ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

typedef enum {
    ILI9341_ROTATION_0   = (ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR),
    ILI9341_ROTATION_90  = (ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR),
    ILI9341_ROTATION_180 = (ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR),
    ILI9341_ROTATION_270 = (ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR),
} ILI9341_Rotation;

void ILI9341_Init(void);
void ILI9341_SetRotation(ILI9341_Rotation rotation);
void ILI9341_SetAddressWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ILI9341_FillScreen(uint16_t color);
void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9341_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9341_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void ILI9341_DrawCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color);
void ILI9341_FillCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color);
void ILI9341_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void ILI9341_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
void ILI9341_SetBacklight(uint8_t on);

void ILI9341_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
void ILI9341_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t size);

void ILI9341_WriteColor(uint16_t color);

#ifdef __cplusplus
}
#endif

#endif
