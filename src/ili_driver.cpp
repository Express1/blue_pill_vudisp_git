#include <Arduino.h>
#include <SPI.h>
#include "ili_driver.h"
#include "font5x7.h"

#define ILI9341_COLADDR   0x2A
#define ILI9341_PAGEADDR  0x2B
#define ILI9341_RAMWR     0x2C
#define ILI9341_MADCTL    0x36
#define ILI9341_PIXFMT    0x3A
#define ILI9341_SLPOUT    0x11
#define ILI9341_DISPON    0x29

#define TFT_DC   PB14 //PB1
#define TFT_CS   PA4 //PB10
#define TFT_RST  PB13 //PB3

#define TFT_WIDTH   320
#define TFT_HEIGHT  240

// BAR_WIDTH derives from the segment count (defined in ili_driver.h), so the two
// always stay in sync — change VU_DB_FLOOR/VU_DB_CEILING there, not here.
#define BAR_WIDTH   (VU_NUM_SEGMENTS * 5)
#define BAR_HEIGHT  8
#define BAR_BYTES   (BAR_WIDTH * BAR_HEIGHT * 2)
#define METRICS_HEIGHT 8
#define METRICS_BYTES (BAR_WIDTH * METRICS_HEIGHT * 2)

#define COLOR_BLACK       0x0000
#define COLOR_RED         0xF800
#define COLOR_GREEN       0x07E0
#define COLOR_YELLOW      0xFFE0
#define COLOR_BLUE        0x001F
#define COLOR_WHITE       0xFFFF
#define VU_COLOR(color) static_cast<uint16_t>( \
    (((((color) >> 11) & 0x1F) * 6 / 6) << 11) | \
    (((((color) >> 5) & 0x3F) * 6 / 6) << 5) | \
    ((((color) & 0x1F) * 4 / 6)) )
#define DARK_COLOR(color) static_cast<uint16_t>( \
    (((((color) >> 11) & 0x1F) * 1 / 4) << 11) | \
    (((((color) >> 5) & 0x3F) * 1 / 4) << 5) | \
    ((((color) & 0x1F) * 1 / 4)) )

// spi dma
volatile bool bar_dma_busy = false;

static uint8_t barTransferBuffer[METRICS_BYTES];

// VU fill: green below -12, yellow -12..-1, red at 0 and above.
static uint16_t vu_level_color(int level)
{
    if (level <= -12) return COLOR_GREEN;
    if (level < 0) return COLOR_YELLOW;
    return COLOR_RED;
}

// Peak marker: green below -12, yellow -12..11, red at 12 and above.
static uint16_t peak_level_color(int level)
{
    if (level <= -12) return COLOR_BLUE;
    if (level < 12) return COLOR_BLUE;
    return COLOR_RED;
}

static void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    y = TFT_HEIGHT - y - h;

    digitalWrite(TFT_CS, LOW);
    digitalWrite(TFT_DC, LOW);
    SPI.transfer(ILI9341_COLADDR);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(x >> 8);
    SPI.transfer(x & 0xFF);
    SPI.transfer((x + w - 1) >> 8);
    SPI.transfer((x + w - 1) & 0xFF);

    digitalWrite(TFT_DC, LOW);
    SPI.transfer(ILI9341_PAGEADDR);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(y >> 8);
    SPI.transfer(y & 0xFF);
    SPI.transfer((y + h - 1) >> 8);
    SPI.transfer((y + h - 1) & 0xFF);

    digitalWrite(TFT_DC, LOW);
    SPI.transfer(ILI9341_RAMWR);
    digitalWrite(TFT_DC, HIGH);
}

void ili_init()
{
    pinMode(TFT_CS, OUTPUT);
    pinMode(TFT_DC, OUTPUT);
    pinMode(TFT_RST, OUTPUT);

    digitalWrite(TFT_CS, HIGH);
    digitalWrite(TFT_DC, HIGH);
    digitalWrite(TFT_RST, LOW);
    delay(20);
    digitalWrite(TFT_RST, HIGH);
    delay(150);

    SPI.begin();
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));

    digitalWrite(TFT_CS, LOW);
    digitalWrite(TFT_DC, LOW);
    SPI.transfer(ILI9341_SLPOUT);
    digitalWrite(TFT_CS, HIGH);
    delay(120);

    digitalWrite(TFT_CS, LOW);
    digitalWrite(TFT_DC, LOW);
    SPI.transfer(ILI9341_PIXFMT);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(0x55);

    digitalWrite(TFT_DC, LOW);
    SPI.transfer(ILI9341_MADCTL);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(0x28);
    digitalWrite(TFT_CS, HIGH);

    digitalWrite(TFT_CS, LOW);
    digitalWrite(TFT_DC, LOW);
    SPI.transfer(ILI9341_DISPON);
    digitalWrite(TFT_CS, HIGH);
}

void ili_fill(uint16_t color)
{
    setAddrWindow(0, 0, TFT_WIDTH, TFT_HEIGHT);
    digitalWrite(TFT_CS, LOW);
    for (uint32_t index = 0; index < (uint32_t)TFT_WIDTH * TFT_HEIGHT; index++) {
        SPI.transfer16(color);
    }
    digitalWrite(TFT_CS, HIGH);
}

void ili_drawString(int x, int y, const char *str, uint16_t color)
{
    while (*str) {
        char character = *str++;
        if (character < 32 || character > 126) continue;

        const uint8_t *glyph = font5x7[character - 32];
        setAddrWindow(TFT_WIDTH - x - 5, y, 5, 7);
        digitalWrite(TFT_CS, LOW);
        for (int row = 0; row < 7; row++) {
            for (int column = 4; column >= 0; column--) {
                uint16_t pixel = (glyph[column] & (1 << (6 - row))) ? color : COLOR_BLACK;
                SPI.transfer16(pixel);
            }
        }
        digitalWrite(TFT_CS, HIGH);
        x += 6;
    }
}

void draw_horizontal_line(int start_x, int end_x, int y, uint16_t color)
{
    if (end_x < start_x) return;
    int width = end_x - start_x + 1;
    setAddrWindow(TFT_WIDTH - end_x - 1, y, width, 1);
    digitalWrite(TFT_CS, LOW);
    for (int index = 0; index < width; index++) SPI.transfer16(color);
    digitalWrite(TFT_CS, HIGH);
}

void draw_vertical_line(int x, int start_y, int end_y, uint16_t color)
{
    if (end_y < start_y) return;
    int height = end_y - start_y + 1;
    setAddrWindow(TFT_WIDTH - x - 1, start_y, 1, height);
    digitalWrite(TFT_CS, LOW);
    for (int index = 0; index < height; index++) SPI.transfer16(color);
    digitalWrite(TFT_CS, HIGH);
}

void create_bar(int vu, int peak)
{
    if (vu < VU_DB_FLOOR) vu = VU_DB_FLOOR;
    if (vu > VU_DB_CEILING) vu = VU_DB_CEILING;
    if (peak < VU_DB_FLOOR) peak = VU_DB_FLOOR;
    if (peak > VU_DB_CEILING) peak = VU_DB_CEILING;

    // At the floor, nothing is lit at all (matches every other inactive segment).
    bool vu_is_floor = (vu <= VU_DB_FLOOR);

    uint16_t segmentColors[VU_NUM_SEGMENTS];
    for (int segment = 0; segment < VU_NUM_SEGMENTS; segment++) {
        int level = segment + VU_DB_FLOOR;
        segmentColors[segment] = (!vu_is_floor && segment <= (vu - VU_DB_FLOOR))
            ? VU_COLOR(vu_level_color(level))
            : DARK_COLOR(vu_level_color(level));
    }

    int peakSegment = peak - VU_DB_FLOOR;
    if (peakSegment >= VU_NUM_SEGMENTS) peakSegment = VU_NUM_SEGMENTS - 1;
    // Don't draw the peak marker on top of the VU bar's own leading edge.
    if (peak != vu) {
        segmentColors[peakSegment] = peak_level_color(peak);
    }

    uint32_t byteIndex = 0;
    for (int row = 0; row < BAR_HEIGHT; row++) {
        for (int wireColumn = 0; wireColumn < BAR_WIDTH; wireColumn++) {
            int logicalColumn = BAR_WIDTH - 1 - wireColumn;
            int segment = logicalColumn / 5;
            uint16_t pixel = (logicalColumn % 5 < 4)
                ? segmentColors[segment]
                : COLOR_BLACK;
            barTransferBuffer[byteIndex++] = pixel >> 8;
            barTransferBuffer[byteIndex++] = pixel & 0xFF;
        }
    }
}

static void send_buffer_dma(int x, int y, int width, int height)
{
    static SPI_HandleTypeDef *hspi1 = nullptr;
    if (hspi1 == nullptr) hspi1 = SPI.getHandle();

    setAddrWindow(x, y, width, height);
    digitalWrite(TFT_CS, LOW);
    bar_dma_busy = true;
    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(
        hspi1, barTransferBuffer, width * height * 2);
    if (status != HAL_OK) {
        bar_dma_busy = false;
        digitalWrite(TFT_CS, HIGH);
        return;
    }

    while (bar_dma_busy);
}

void send_bar(int x, int y)
{
    send_buffer_dma(x, y, BAR_WIDTH, BAR_HEIGHT);
}

static int draw_buffer_text(int x, int y, const char *text, uint16_t color)
{
    while (*text) {
        char character = *text++;
        if (character < 32 || character > 126) continue;

        const uint8_t *glyph = font5x7[character - 32];
        for (int row = 0; row < 7; row++) {
            for (int column = 0; column < 5; column++) {
                if (glyph[column] & (1 << (6 - row))) {
                    int logicalColumn = x + column;
                    int wireColumn = BAR_WIDTH - 1 - logicalColumn;
                    uint32_t byteIndex = (y + row) * BAR_WIDTH * 2 + wireColumn * 2;
                    barTransferBuffer[byteIndex] = color >> 8;
                    barTransferBuffer[byteIndex + 1] = color & 0xFF;
                }
            }
        }
        x += 6;
    }
    return x;
}

static int draw_buffer_unsigned(int x, int y, uint32_t value, uint16_t color)
{
    char valueText[11];
    snprintf(valueText, sizeof(valueText), "%lu", static_cast<unsigned long>(value));
    return draw_buffer_text(x, y, valueText, color);
}

static int draw_buffer_db(int x, int y, int value, int threshold, uint16_t belowColor)
{
    unsigned int magnitude = static_cast<unsigned int>(value < 0 ? -value : value);
    char valueText[11];
    snprintf(valueText, sizeof(valueText), "%u", magnitude);
    uint16_t color = value < threshold ? belowColor : COLOR_RED;
    return draw_buffer_text(x, y, valueText, color);
}

static int draw_audio_metrics_values(int x, uint32_t raw_rms, int vu_db, int pk_db)
{
    x = draw_buffer_unsigned(x, 0, raw_rms, COLOR_WHITE);
    x += 6;
    x = draw_buffer_db(x, 0, vu_db, 0, COLOR_GREEN);
    x += 6;
    return draw_buffer_db(x, 0, pk_db, 12, COLOR_BLUE);
}

void draw_audio_metrics(int x, int y,
                        uint32_t raw_rms_ch0, int vu_db_ch0, int pk_db_ch0,
                        uint32_t raw_rms_ch1, int vu_db_ch1, int pk_db_ch1,
                        uint32_t frame_work_percent)
{
    for (uint32_t byteIndex = 0; byteIndex < METRICS_BYTES; byteIndex++) {
        barTransferBuffer[byteIndex] = 0;
    }

    int cursor = draw_audio_metrics_values(0, raw_rms_ch0, vu_db_ch0, pk_db_ch0);
    cursor = draw_buffer_text(cursor + 6, 0, "   ", COLOR_WHITE);
    cursor = draw_audio_metrics_values(cursor, raw_rms_ch1, vu_db_ch1, pk_db_ch1);
    cursor = draw_buffer_unsigned(cursor + 6, 0, frame_work_percent, COLOR_WHITE);
    draw_buffer_text(cursor, 0, "%", COLOR_WHITE);

    send_buffer_dma(x, y, BAR_WIDTH, METRICS_HEIGHT);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    digitalWrite(TFT_CS, HIGH);
    bar_dma_busy = false;
}

static DMA_HandleTypeDef hdma_spi1_tx;

void setup_spi_dma()
{
    SPI_HandleTypeDef *hspi1 = SPI.getHandle();

    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_spi1_tx.Instance = DMA1_Channel3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&hdma_spi1_tx);

    __HAL_LINKDMA(hspi1, hdmatx, hdma_spi1_tx);

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0); // lower priority than ADC's (0,0)
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

extern "C" void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_tx);
}