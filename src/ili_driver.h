#ifndef ILI_DRIVER_H
#define ILI_DRIVER_H

#include <Arduino.h>

// VU scale range — change these two and everything else (segment count, bar width) follows.
#define VU_DB_FLOOR      (-40)
#define VU_DB_CEILING    14
#define VU_NUM_SEGMENTS  (VU_DB_CEILING - VU_DB_FLOOR + 1)   // 55
#define VU_NEW_DB_FLOOR  (-30)

void ili_init();
void setup_spi_dma();
void ili_set_display_mode(bool new_display);
void ili_fill(uint16_t color);
void ili_drawString(int x, int y, const char* s, uint16_t color);

void draw_horizontal_line(int start_x, int end_x, int y, uint16_t color);
void draw_vertical_line(int x, int start_y, int end_y, uint16_t color);

void create_bar(int vu, int peak);
void send_bar(int x, int y);
void draw_audio_metrics(int x, int y,
                        uint32_t raw_rms_ch0, int vu_db_ch0, int pk_db_ch0,
                        uint32_t raw_rms_ch1, int vu_db_ch1, int pk_db_ch1,
                        uint32_t frame_work_percent);

#endif