// Startup display selection: PB5/PB4 use INPUT_PULLUP and are sampled once.
// 11 selects the existing -40..+14 display; 10 selects the new -30..+14 display.
// 01 and 00 are reserved and leave the display blank.
#include "Arduino.h"
#include "ili_driver.h"
#include <SPI.h>

#define SAMPLES_PER_BLOCK 120
#define TOTAL_BUF_LEN (SAMPLES_PER_BLOCK * 2)
#define BLOCKS_PER_DISPLAY 64
#define TOTAL_SAMPLES (SAMPLES_PER_BLOCK * BLOCKS_PER_DISPLAY) // 7,680 samples
// turn off display if the audio is silent for this many seconds
#define DISPLAY_OFF_SILENCE_SECONDS 10
#define AUDIO_FRAMES_PER_SECOND 25
#define AUDIO_FRAMES_REQUIRED 20

// ==========================================================
// PARAMETERIZED BALLISTICS & INTERACTION TUNING REGISTER MAP
// ==========================================================
// Base display execution cadence: 25Hz (1 Frame = 40ms)
// Block (ISR) cadence: 120 samples @ 192kHz = 625us per block

// Hardware IO Bindings
#define AUDIO_IN_L_PIN PA0    // Mapped directly to ADC1 Master
#define AUDIO_IN_R_PIN PA1    // Mapped directly to ADC2 Slave
#define LCD_BACKLIGHT_PIN PA8 // Drives the LCD backlight: HIGH on, LOW off
// Startup display selection: PB5/PB4 use INPUT_PULLUP and are sampled once.
// 11 selects the existing -40..+14 display; 10 selects the new -30..+14 display.
// 01 and 00 are reserved and leave the display blank.
#define DISPLAY_SELECT_PB5 PB5
#define DISPLAY_SELECT_PB4 PB4

// Peak decay is applied at the 40ms display rate, scaled to the 54dB display
// range: 1.7s * (54/20) = 4.59s full-scale fall.
#define PEAK_HOLD_SECONDS 0.0f // time peak holds before decay starts; 0.0f = pure PPM spec (no hold)
#define PEAK_FALL_MS 4590      // IEC 60268-10 Type I: 20dB/1.7s, scaled to 54dB

#define PK_DECAY_FRAC_BITS 8
const int32_t CONFIG_PK_DECAY_STEP_FRAC =
    (int32_t)(((int64_t)54 * 1000 * (1 << PK_DECAY_FRAC_BITS)) / ((int64_t)PEAK_FALL_MS * 25));

int32_t pk_decay_accum_ch0 = 0;
int32_t pk_decay_accum_ch1 = 0;
int32_t pk_decay_accum_sum = 0;
int32_t pk_decay_accum_diff = 0;

// Automated Core Multipliers Derived From Target Time Configurations
const uint32_t CONFIG_PK_HOLD_FRAMES = (uint32_t)(PEAK_HOLD_SECONDS * 25.0f);

// Two-stage rectified-average detector: reaches 99% in 300ms at 625us per block.
#define VU_RECTIFIED_ALPHA_Q16 899
#define VU_RECTIFIED_FRAC_BITS 8
#define VU_RECTIFIED_TO_PEAK_Q12 6434

// ==========================================================
// SCREEN LAYOUT — move a whole block by changing ONE number
// ==========================================================
#define L_R_HEIGHT 30
#define S_D_HEIGHT 120
#define NEW_L_BAR_Y 80
#define NEW_R_BAR_Y 103
#define NEW_METRICS_Y 195

#define BLOCK_TOP_LABEL_OFF 0
#define BLOCK_TOP_LINE_OFF 14
#define BLOCK_TOP_TICK_OFF 11
#define BLOCK_ROW1_OFF 18
#define BLOCK_ROW2_OFF 32
#define BLOCK_BOTTOM_LABEL_OFF 51
#define BLOCK_BOTTOM_LINE_OFF 44
#define BLOCK_BOTTOM_TICK_OFF 45
#define BLOCK_HEADER_OFF 61

// Hardware Peripheral Buffers & Status Registers
volatile uint32_t adc_dma_buffer[TOTAL_BUF_LEN];

// silence detector
static bool last_frame_was_silent = false;

// High-Precision Fixed-Point DC Center Tracking Estimators (Scaled by 64 via << 6)
uint32_t dc_accum_ch0 = 131072; // Initialized to midscale 2048 * 64
uint32_t dc_accum_ch1 = 131072;

struct FrameMetrics
{
  uint64_t sum_squares_ch0;
  uint64_t sum_squares_ch1;
  uint64_t sum_squares_sum;
  uint64_t sum_squares_diff;
  uint32_t max_peak_ch0;
  uint32_t max_peak_ch1;
  uint32_t max_peak_sum;
  uint32_t max_peak_diff;
};

volatile FrameMetrics metric_buffers[2] = {};
volatile uint8_t active_metrics_index = 0;
volatile uint8_t ready_metrics_index = 0;
volatile bool frame_metrics_ready = false;

// Cascaded low-pass stages emulate the inertia of a mechanical VU movement.
uint32_t vu_rectified_stage1_ch0_q8 = 0;
uint32_t vu_rectified_stage1_ch1_q8 = 0;
uint32_t vu_rectified_stage1_sum_q8 = 0;
uint32_t vu_rectified_stage1_diff_q8 = 0;
uint32_t vu_rectified_stage2_ch0_q8 = 0;
uint32_t vu_rectified_stage2_ch1_q8 = 0;
uint32_t vu_rectified_stage2_sum_q8 = 0;
uint32_t vu_rectified_stage2_diff_q8 = 0;

// Active decibel storage values for loop persistence
int16_t pk_db_ch0 = VU_DB_FLOOR;
int16_t pk_db_ch1 = VU_DB_FLOOR;
int16_t pk_db_sum = VU_DB_FLOOR;
int16_t pk_db_diff = VU_DB_FLOOR;
uint32_t pk_hold_timer_ch0 = 0;
uint32_t pk_hold_timer_ch1 = 0;
uint32_t pk_hold_timer_sum = 0;
uint32_t pk_hold_timer_diff = 0;
uint32_t frame_block_counter = 0;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim3;

#define COLOR_ORANGE 0xFD20
#define COLOR_RED 0xF800
#define COLOR_YELLOW 0xFFE0
#define COLOR_BLUE 0x001F

static bool display_mode_selected = false;
static bool new_display_mode = false;

extern "C" void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

static void draw_scale(int label_y, int line_y, int tick_start_y)
{
  static const int label_x[] = {18, 68, 118, 168, 199, 209, 225, 240, 250, 272};
  static const int tick_x[] = {27, 77, 127, 177, 201, 211, 227, 242, 252, 277};
  static const char *labels[] = {"-40", "-30", "-20", "-10", "5", "3", "0", "3", "5", "10"};
  static const uint16_t colors[] = {
      COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_RED};

  draw_horizontal_line(25, 219, line_y, COLOR_YELLOW);
  draw_horizontal_line(220, 234, line_y, COLOR_RED);
  draw_horizontal_line(235, 269, line_y, COLOR_YELLOW);
  draw_horizontal_line(270, 299, line_y, COLOR_RED);

  for (int index = 0; index < 10; index++)
  {
    ili_drawString(label_x[index], label_y, labels[index], colors[index]);
    draw_vertical_line(tick_x[index], tick_start_y, tick_start_y + 2, colors[index]);
  }
}

static void draw_new_scale(int top_label_y, int line_y, int tick_start_y,
                           bool ticks_above_line)
{
  draw_horizontal_line(1, 210, line_y, COLOR_YELLOW);
  draw_horizontal_line(211, 224, line_y, COLOR_RED);
  draw_horizontal_line(225, 273, line_y, COLOR_YELLOW);
  draw_horizontal_line(274, 315, line_y, COLOR_RED);

  for (int value = VU_NEW_DB_FLOOR; value <= VU_DB_CEILING; value++)
  {
    const int segment = value - VU_NEW_DB_FLOOR;
    const int label_x = segment * 7 + 5;
    const int tick_x = segment * 7 + 7;
    const uint16_t color = value == 0 || value >= 9 ? COLOR_RED : COLOR_YELLOW;
    const int magnitude = value < 0 ? -value : value;
    const bool lower_label_row = ticks_above_line
        ? value % 2 == 0
        : value % 2 != 0;
    const int label_y = top_label_y + (lower_label_row ? 27 : 0) +
                        (ticks_above_line ? (value % 2 == 0 ? 6 : 8)
                                          : (value % 2 == 0 ? 2 : 0));
    if (magnitude >= 10)
    {
      const char tens[] = {static_cast<char>('0' + magnitude / 10), '\0'};
      ili_drawString(label_x, label_y, tens, color);
    }

    const char ones[] = {static_cast<char>('0' + magnitude % 10), '\0'};
    ili_drawString(label_x, label_y + 10, ones, color);

    if (ticks_above_line)
    {
      draw_vertical_line(tick_x, tick_start_y - (value % 2 != 0 ? 25 : 0),
                         tick_start_y + 2, color);
    }
    else
    {
      draw_vertical_line(tick_x, tick_start_y,
                         tick_start_y + (value % 2 != 0 ? 27 : 2), color);
    }
  }
}

static void draw_new_layout()
{
  draw_new_scale(16, 74, 71, true);
  draw_new_scale(132, 125, 126, false);
  ili_drawString(0, 34, "L", COLOR_YELLOW);
  ili_drawString(0, 159, "R", COLOR_YELLOW);
  ili_drawString(136, 182, "VU RANGE", COLOR_YELLOW);
  ili_drawString(215, 182, "|", COLOR_RED);
  ili_drawString(250, 182, "PPM", COLOR_BLUE);
}

static void draw_block(int top_y, const char *row1_label, const char *row2_label)
{
  draw_scale(top_y + BLOCK_TOP_LABEL_OFF, top_y + BLOCK_TOP_LINE_OFF, top_y + BLOCK_TOP_TICK_OFF);
  ili_drawString(7, top_y + BLOCK_ROW1_OFF, row1_label, COLOR_YELLOW);
  ili_drawString(7, top_y + BLOCK_ROW2_OFF, row2_label, COLOR_YELLOW);
  draw_scale(top_y + BLOCK_BOTTOM_LABEL_OFF, top_y + BLOCK_BOTTOM_LINE_OFF, top_y + BLOCK_BOTTOM_TICK_OFF);

  int header_y = top_y + BLOCK_HEADER_OFF;
  ili_drawString(136, header_y, "VU RANGE", COLOR_YELLOW);
  ili_drawString(225, header_y, "|", COLOR_RED);
  ili_drawString(250, header_y, "PPM", COLOR_BLUE);
}

void setup()
{
  Serial.begin(115200);

  pinMode(DISPLAY_SELECT_PB5, INPUT_PULLUP);
  pinMode(DISPLAY_SELECT_PB4, INPUT_PULLUP);
  const bool pb5_high = digitalRead(DISPLAY_SELECT_PB5) == HIGH;
  const bool pb4_high = digitalRead(DISPLAY_SELECT_PB4) == HIGH;
  display_mode_selected = pb5_high;
  new_display_mode = pb5_high && !pb4_high;
  ili_set_display_mode(new_display_mode);

  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);

  ili_init();
  setup_spi_dma();

  ili_fill(0x0000);
  if (display_mode_selected)
  {
    if (new_display_mode)
    {
      draw_new_layout();
    }
    else
    {
      draw_block(L_R_HEIGHT, "L", "R");
      draw_block(S_D_HEIGHT, "S", "D");
    }
  }

  // 1. Clock Power Declarations
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&hdma_adc1);
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc1);

  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc2);

  ADC_MultiModeTypeDef multimode = {0};
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 374;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);

  SET_BIT(hadc2.Instance->CR2, ADC_CR2_DMA);
  __HAL_ADC_ENABLE(&hadc2);
  HAL_ADC_Start(&hadc2);

  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_dma_buffer, TOTAL_BUF_LEN);
  HAL_TIM_Base_Start(&htim3);
}

static inline void update_rectified_average(uint32_t target_q8, uint32_t &average_q8)
{
  if (target_q8 > average_q8)
  {
    average_q8 += ((target_q8 - average_q8) * VU_RECTIFIED_ALPHA_Q16 + 32768) >> 16;
  }
  else
  {
    average_q8 -= ((average_q8 - target_q8) * VU_RECTIFIED_ALPHA_Q16 + 32768) >> 16;
  }
}

static void process_dma_block(uint32_t source_offset)
{
  FrameMetrics &metrics = const_cast<FrameMetrics &>(metric_buffers[active_metrics_index]);

  uint32_t tracked_dc_ch0 = dc_accum_ch0 >> 6;
  uint32_t tracked_dc_ch1 = dc_accum_ch1 >> 6;

  uint32_t block_sum_ch0 = 0;
  uint32_t block_sum_ch1 = 0;

  uint64_t sum_sq_ch0 = 0, sum_sq_ch1 = 0, sum_sq_sum = 0, sum_sq_diff = 0;
  uint32_t sum_abs_ch0 = 0, sum_abs_ch1 = 0, sum_abs_sum = 0, sum_abs_diff = 0;
  uint32_t frame_peak_ch0 = metrics.max_peak_ch0;
  uint32_t frame_peak_ch1 = metrics.max_peak_ch1;
  uint32_t frame_peak_sum = metrics.max_peak_sum;
  uint32_t frame_peak_diff = metrics.max_peak_diff;
  uint32_t block_peak_ch0 = 0, block_peak_ch1 = 0, block_peak_sum = 0, block_peak_diff = 0;

  for (uint32_t index = 0; index < SAMPLES_PER_BLOCK; index++)
  {
    uint32_t packed_value = adc_dma_buffer[source_offset + index];
    uint32_t raw_ch0 = packed_value & 0xFFFF;
    uint32_t raw_ch1 = packed_value >> 16;
    block_sum_ch0 += raw_ch0;
    block_sum_ch1 += raw_ch1;

    int32_t ac_ch0 = (int32_t)raw_ch0 - (int32_t)tracked_dc_ch0;
    int32_t ac_ch1 = (int32_t)raw_ch1 - (int32_t)tracked_dc_ch1;
    int32_t ac_sum = (ac_ch0 + ac_ch1) / 2;
    int32_t ac_diff = (ac_ch0 - ac_ch1) / 2;

    uint32_t abs_ch0 = ac_ch0 < 0 ? -ac_ch0 : ac_ch0;
    uint32_t abs_ch1 = ac_ch1 < 0 ? -ac_ch1 : ac_ch1;
    uint32_t abs_sum = ac_sum < 0 ? -ac_sum : ac_sum;
    uint32_t abs_diff = ac_diff < 0 ? -ac_diff : ac_diff;

    sum_sq_ch0 += (int64_t)ac_ch0 * ac_ch0;
    sum_sq_ch1 += (int64_t)ac_ch1 * ac_ch1;
    sum_sq_sum += (int64_t)ac_sum * ac_sum;
    sum_sq_diff += (int64_t)ac_diff * ac_diff;
    sum_abs_ch0 += abs_ch0;
    sum_abs_ch1 += abs_ch1;
    sum_abs_sum += abs_sum;
    sum_abs_diff += abs_diff;

    if (abs_ch0 > block_peak_ch0)
      block_peak_ch0 = abs_ch0;
    if (abs_ch1 > block_peak_ch1)
      block_peak_ch1 = abs_ch1;
    if (abs_sum > block_peak_sum)
      block_peak_sum = abs_sum;
    if (abs_diff > block_peak_diff)
      block_peak_diff = abs_diff;
  }

  if (block_peak_ch0 > frame_peak_ch0)
    frame_peak_ch0 = block_peak_ch0;
  if (block_peak_ch1 > frame_peak_ch1)
    frame_peak_ch1 = block_peak_ch1;
  if (block_peak_sum > frame_peak_sum)
    frame_peak_sum = block_peak_sum;
  if (block_peak_diff > frame_peak_diff)
    frame_peak_diff = block_peak_diff;

  metrics.max_peak_ch0 = frame_peak_ch0;
  metrics.max_peak_ch1 = frame_peak_ch1;
  metrics.max_peak_sum = frame_peak_sum;
  metrics.max_peak_diff = frame_peak_diff;

  dc_accum_ch0 = dc_accum_ch0 - (dc_accum_ch0 >> 6) + block_sum_ch0 / SAMPLES_PER_BLOCK;
  dc_accum_ch1 = dc_accum_ch1 - (dc_accum_ch1 >> 6) + block_sum_ch1 / SAMPLES_PER_BLOCK;

  metrics.sum_squares_ch0 += sum_sq_ch0;
  metrics.sum_squares_ch1 += sum_sq_ch1;
  metrics.sum_squares_sum += sum_sq_sum;
  metrics.sum_squares_diff += sum_sq_diff;

  uint32_t block_rectified_ch0_q8 = (sum_abs_ch0 << VU_RECTIFIED_FRAC_BITS) / SAMPLES_PER_BLOCK;
  uint32_t block_rectified_ch1_q8 = (sum_abs_ch1 << VU_RECTIFIED_FRAC_BITS) / SAMPLES_PER_BLOCK;
  uint32_t block_rectified_sum_q8 = (sum_abs_sum << VU_RECTIFIED_FRAC_BITS) / SAMPLES_PER_BLOCK;
  uint32_t block_rectified_diff_q8 = (sum_abs_diff << VU_RECTIFIED_FRAC_BITS) / SAMPLES_PER_BLOCK;
  update_rectified_average(block_rectified_ch0_q8, vu_rectified_stage1_ch0_q8);
  update_rectified_average(block_rectified_ch1_q8, vu_rectified_stage1_ch1_q8);
  update_rectified_average(block_rectified_sum_q8, vu_rectified_stage1_sum_q8);
  update_rectified_average(block_rectified_diff_q8, vu_rectified_stage1_diff_q8);
  update_rectified_average(vu_rectified_stage1_ch0_q8, vu_rectified_stage2_ch0_q8);
  update_rectified_average(vu_rectified_stage1_ch1_q8, vu_rectified_stage2_ch1_q8);
  update_rectified_average(vu_rectified_stage1_sum_q8, vu_rectified_stage2_sum_q8);
  update_rectified_average(vu_rectified_stage1_diff_q8, vu_rectified_stage2_diff_q8);

  frame_block_counter++;
  if (frame_block_counter >= BLOCKS_PER_DISPLAY)
  {
    uint8_t completed_index = active_metrics_index;
    active_metrics_index ^= 1;
    metric_buffers[active_metrics_index].sum_squares_ch0 = 0;
    metric_buffers[active_metrics_index].sum_squares_ch1 = 0;
    metric_buffers[active_metrics_index].sum_squares_sum = 0;
    metric_buffers[active_metrics_index].sum_squares_diff = 0;
    metric_buffers[active_metrics_index].max_peak_ch0 = 0;
    metric_buffers[active_metrics_index].max_peak_ch1 = 0;
    metric_buffers[active_metrics_index].max_peak_sum = 0;
    metric_buffers[active_metrics_index].max_peak_diff = 0;
    ready_metrics_index = completed_index;
    frame_metrics_ready = true;
    frame_block_counter = 0;
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  process_dma_block(0);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  process_dma_block(SAMPLES_PER_BLOCK);
}

uint32_t integer_sqrt(uint64_t n)
{
  if (n == 0)
    return 0;
  uint64_t x = n;
  uint64_t y = (x + 1) >> 1;
  while (y < x)
  {
    x = y;
    y = (x + n / x) >> 1;
  }
  return (uint32_t)x;
}

// Binary search table lookup for 45 discrete dB values from VU_DB_FLOOR to VU_DB_CEILING
int16_t value_to_db(uint32_t val) {
    // 54 thresholds, continuous coverage from VU_DB_FLOOR+1 (-39) through VU_DB_CEILING (+14).
    // First 9 entries (-39..-31) are extrapolated to close the old table's dead zone;
    // the remaining 45 (-30..+13) are the original, unchanged table.
    static const uint16_t t[] = {
      4, 5, 6, 7, 8, 9, 10, 11, 12,                                        // -39..-31 (new)
      13, 15, 16, 18, 21, 23, 26, 29, 33, 37, 41, 46, 52, 58, 65, 73,
      82, 92, 104, 116, 130, 146, 164, 184, 207, 232, 260, 292, 327,
      367, 412, 462, 519, 582, 653, 733, 822, 922, 1035, 1161, 1303,
      1462, 1640, 1841, 2065
    };
    const int N = sizeof(t) / sizeof(t[0]);   // 54

    if (val < t[0]) return VU_DB_FLOOR;         // true silence
    if (val >= t[N - 1]) return VU_DB_CEILING;  // hard ceiling

    int lo = 1, hi = N - 1;
    while (lo < hi) {
        int mid = (lo + hi) >> 1;
        if (val < t[mid]) hi = mid;
        else lo = mid + 1;
    }

    return (VU_DB_FLOOR + 1) + (lo - 1);   // t[0] → -39, climbing by 1 per bin
}


void loop()
{

  static uint32_t frame_work_total_us = 0;
  static uint32_t frame_work_percent = 0;
  static uint32_t audio_window_start_ms = 0;
  static uint8_t silent_frames_in_window = 0;
  static uint8_t active_frames_in_window = 0;
  static uint16_t consecutive_silent_seconds = 0;
  static bool display_is_on = true;
  static bool audio_window_started = false;

  if (!audio_window_started)
  {
    audio_window_start_ms = millis();
    audio_window_started = true;
  }

  FrameMetrics frame = {};
  uint32_t vu_rectified_ch0_q8 = 0;
  uint32_t vu_rectified_ch1_q8 = 0;
  uint32_t vu_rectified_sum_q8 = 0;
  uint32_t vu_rectified_diff_q8 = 0;
  bool display_frame_triggered = false;

  noInterrupts();
  uint32_t reading0 = micros();
  uint32_t frame_work_us = 0;

  if (frame_metrics_ready)
  {
    uint8_t ready_index = ready_metrics_index;
    frame.sum_squares_ch0 = metric_buffers[ready_index].sum_squares_ch0;
    frame.sum_squares_ch1 = metric_buffers[ready_index].sum_squares_ch1;
    frame.sum_squares_sum = metric_buffers[ready_index].sum_squares_sum;
    frame.sum_squares_diff = metric_buffers[ready_index].sum_squares_diff;
    frame.max_peak_ch0 = metric_buffers[ready_index].max_peak_ch0;
    frame.max_peak_ch1 = metric_buffers[ready_index].max_peak_ch1;
    frame.max_peak_sum = metric_buffers[ready_index].max_peak_sum;
    frame.max_peak_diff = metric_buffers[ready_index].max_peak_diff;
    vu_rectified_ch0_q8 = vu_rectified_stage2_ch0_q8;
    vu_rectified_ch1_q8 = vu_rectified_stage2_ch1_q8;
    vu_rectified_sum_q8 = vu_rectified_stage2_sum_q8;
    vu_rectified_diff_q8 = vu_rectified_stage2_diff_q8;
    frame_metrics_ready = false;
    display_frame_triggered = true;
  }
  interrupts();

  if (display_frame_triggered)
  {
    uint32_t raw_rms_ch0 = integer_sqrt(frame.sum_squares_ch0 / TOTAL_SAMPLES);
    uint32_t raw_rms_ch1 = integer_sqrt(frame.sum_squares_ch1 / TOTAL_SAMPLES);
    uint32_t raw_rms_sum = integer_sqrt(frame.sum_squares_sum / TOTAL_SAMPLES);
    uint32_t raw_rms_diff = integer_sqrt(frame.sum_squares_diff / TOTAL_SAMPLES);

    if (raw_rms_ch0 <= 2)
      raw_rms_ch0 = 0;
    if (raw_rms_ch1 <= 2)
      raw_rms_ch1 = 0;
    if (raw_rms_sum <= 2)
      raw_rms_sum = 0;
    if (raw_rms_diff <= 2)
      raw_rms_diff = 0;

    uint32_t calibrated_vu_ch0 = (uint32_t)(((uint64_t)vu_rectified_ch0_q8 * VU_RECTIFIED_TO_PEAK_Q12 + (1ULL << 19)) >> 20);
    uint32_t calibrated_vu_ch1 = (uint32_t)(((uint64_t)vu_rectified_ch1_q8 * VU_RECTIFIED_TO_PEAK_Q12 + (1ULL << 19)) >> 20);
    uint32_t calibrated_vu_sum = (uint32_t)(((uint64_t)vu_rectified_sum_q8 * VU_RECTIFIED_TO_PEAK_Q12 + (1ULL << 19)) >> 20);
    uint32_t calibrated_vu_diff = (uint32_t)(((uint64_t)vu_rectified_diff_q8 * VU_RECTIFIED_TO_PEAK_Q12 + (1ULL << 19)) >> 20);

    int16_t final_vu_db_ch0 = value_to_db(calibrated_vu_ch0);
    int16_t final_vu_db_ch1 = value_to_db(calibrated_vu_ch1);
    int16_t final_vu_db_sum = value_to_db(calibrated_vu_sum);
    int16_t final_vu_db_diff = value_to_db(calibrated_vu_diff);

    uint32_t peak_ch0_for_display = (frame.max_peak_ch0 <= 3) ? 0 : frame.max_peak_ch0;
    uint32_t peak_ch1_for_display = (frame.max_peak_ch1 <= 3) ? 0 : frame.max_peak_ch1;
    uint32_t peak_sum_for_display = (frame.max_peak_sum <= 3) ? 0 : frame.max_peak_sum;
    uint32_t peak_diff_for_display = (frame.max_peak_diff <= 3) ? 0 : frame.max_peak_diff;

    int16_t inst_pk_db_ch0 = value_to_db(peak_ch0_for_display);
    int16_t inst_pk_db_ch1 = value_to_db(peak_ch1_for_display);
    int16_t inst_pk_db_sum = value_to_db(peak_sum_for_display);
    int16_t inst_pk_db_diff = value_to_db(peak_diff_for_display);

    if (final_vu_db_ch0 == VU_DB_FLOOR)
      inst_pk_db_ch0 = VU_DB_FLOOR;
    if (final_vu_db_ch1 == VU_DB_FLOOR)
      inst_pk_db_ch1 = VU_DB_FLOOR;
    if (final_vu_db_sum == VU_DB_FLOOR)
      inst_pk_db_sum = VU_DB_FLOOR;
    if (final_vu_db_diff == VU_DB_FLOOR)
      inst_pk_db_diff = VU_DB_FLOOR;

    // Retain the strongest measured peak and decay it between stronger peaks.
    if (inst_pk_db_ch0 >= pk_db_ch0)
    {
      pk_db_ch0 = inst_pk_db_ch0;
      pk_hold_timer_ch0 = CONFIG_PK_HOLD_FRAMES;
      pk_decay_accum_ch0 = 0;
    }
    else
    {
      if (pk_hold_timer_ch0 > 0)
      {
        pk_hold_timer_ch0--;
      }
      else if (pk_db_ch0 > VU_DB_FLOOR)
      {
        pk_decay_accum_ch0 += CONFIG_PK_DECAY_STEP_FRAC;
        int32_t whole_step = pk_decay_accum_ch0 >> PK_DECAY_FRAC_BITS;
        if (whole_step > 0)
        {
          pk_decay_accum_ch0 -= whole_step << PK_DECAY_FRAC_BITS;
          pk_db_ch0 -= whole_step;
          if (pk_db_ch0 < VU_DB_FLOOR)
          {
            pk_db_ch0 = VU_DB_FLOOR;
            pk_decay_accum_ch0 = 0;
          }
        }
      }
    }

    if (inst_pk_db_ch1 >= pk_db_ch1)
    {
      pk_db_ch1 = inst_pk_db_ch1;
      pk_hold_timer_ch1 = CONFIG_PK_HOLD_FRAMES;
      pk_decay_accum_ch1 = 0;
    }
    else
    {
      if (pk_hold_timer_ch1 > 0)
      {
        pk_hold_timer_ch1--;
      }
      else if (pk_db_ch1 > VU_DB_FLOOR)
      {
        pk_decay_accum_ch1 += CONFIG_PK_DECAY_STEP_FRAC;
        int32_t whole_step = pk_decay_accum_ch1 >> PK_DECAY_FRAC_BITS;
        if (whole_step > 0)
        {
          pk_decay_accum_ch1 -= whole_step << PK_DECAY_FRAC_BITS;
          pk_db_ch1 -= whole_step;
          if (pk_db_ch1 < VU_DB_FLOOR)
          {
            pk_db_ch1 = VU_DB_FLOOR;
            pk_decay_accum_ch1 = 0;
          }
        }
      }
    }

    if (inst_pk_db_sum >= pk_db_sum)
    {
      pk_db_sum = inst_pk_db_sum;
      pk_hold_timer_sum = CONFIG_PK_HOLD_FRAMES;
      pk_decay_accum_sum = 0;
    }
    else
    {
      if (pk_hold_timer_sum > 0)
      {
        pk_hold_timer_sum--;
      }
      else if (pk_db_sum > VU_DB_FLOOR)
      {
        pk_decay_accum_sum += CONFIG_PK_DECAY_STEP_FRAC;
        int32_t whole_step = pk_decay_accum_sum >> PK_DECAY_FRAC_BITS;
        if (whole_step > 0)
        {
          pk_decay_accum_sum -= whole_step << PK_DECAY_FRAC_BITS;
          pk_db_sum -= whole_step;
          if (pk_db_sum < VU_DB_FLOOR)
          {
            pk_db_sum = VU_DB_FLOOR;
            pk_decay_accum_sum = 0;
          }
        }
      }
    }

    if (inst_pk_db_diff >= pk_db_diff)
    {
      pk_db_diff = inst_pk_db_diff;
      pk_hold_timer_diff = CONFIG_PK_HOLD_FRAMES;
      pk_decay_accum_diff = 0;
    }
    else
    {
      if (pk_hold_timer_diff > 0)
      {
        pk_hold_timer_diff--;
      }
      else if (pk_db_diff > VU_DB_FLOOR)
      {
        pk_decay_accum_diff += CONFIG_PK_DECAY_STEP_FRAC;
        int32_t whole_step = pk_decay_accum_diff >> PK_DECAY_FRAC_BITS;
        if (whole_step > 0)
        {
          pk_decay_accum_diff -= whole_step << PK_DECAY_FRAC_BITS;
          pk_db_diff -= whole_step;
          if (pk_db_diff < VU_DB_FLOOR)
          {
            pk_db_diff = VU_DB_FLOOR;
            pk_decay_accum_diff = 0;
          }
        }
      }
    }

    bool is_silent_now =
        (final_vu_db_ch0 == VU_DB_FLOOR && pk_db_ch0 == VU_DB_FLOOR) &&
        (final_vu_db_ch1 == VU_DB_FLOOR && pk_db_ch1 == VU_DB_FLOOR) &&
        (final_vu_db_sum == VU_DB_FLOOR && pk_db_sum == VU_DB_FLOOR) &&
        (final_vu_db_diff == VU_DB_FLOOR && pk_db_diff == VU_DB_FLOOR);

    // Count one-second windows of audio frames. A window is considered
    // silent or active only when at least 20 of its nominal 25 frames agree.
    if (is_silent_now)
    {
      silent_frames_in_window++;
    }
    else
    {
      active_frames_in_window++;
    }

    uint32_t now_ms = millis();
    if (now_ms - audio_window_start_ms >= 1000)
    {
      if (display_is_on)
      {
        if (silent_frames_in_window >= AUDIO_FRAMES_REQUIRED)
        {
          consecutive_silent_seconds++;
        }
        else
        {
          consecutive_silent_seconds = 0;
        }

        if (consecutive_silent_seconds >= DISPLAY_OFF_SILENCE_SECONDS)
        {
          // LCD Backlight off, Clear the LCD
          Serial.println("Display off");
          digitalWrite(LCD_BACKLIGHT_PIN, LOW);
          ili_fill(0x0000);
          display_is_on = false;
        }
      }
      else if (active_frames_in_window >= AUDIO_FRAMES_REQUIRED)
      {
        // This is the first serial message after an off period.
        Serial.println("Display on");
        if (display_mode_selected)
        {
          if (new_display_mode)
          {
            draw_new_layout();
          }
          else
          {
            draw_block(L_R_HEIGHT, "L", "R");
            draw_block(S_D_HEIGHT, "S", "D");
          }
        }
        digitalWrite(LCD_BACKLIGHT_PIN, HIGH);
        display_is_on = true;
        consecutive_silent_seconds = 0;
      }

      audio_window_start_ms = now_ms;
      silent_frames_in_window = 0;
      active_frames_in_window = 0;
    }

    bool should_update_display = !(is_silent_now && last_frame_was_silent);

    if (display_mode_selected && display_is_on && should_update_display)
    {
      create_bar(final_vu_db_ch0, pk_db_ch0);
      send_bar(new_display_mode ? 0 : 20,
               new_display_mode ? NEW_L_BAR_Y : L_R_HEIGHT + BLOCK_ROW1_OFF);
      create_bar(final_vu_db_ch1, pk_db_ch1);
      send_bar(new_display_mode ? 0 : 20,
               new_display_mode ? NEW_R_BAR_Y : L_R_HEIGHT + BLOCK_ROW2_OFF);
      if (!new_display_mode)
      {
        create_bar(final_vu_db_sum, pk_db_sum);
        send_bar(20, S_D_HEIGHT + BLOCK_ROW1_OFF);
        create_bar(final_vu_db_diff, pk_db_diff);
        send_bar(20, S_D_HEIGHT + BLOCK_ROW2_OFF);
      }
    }

    last_frame_was_silent = is_silent_now;

        if (!(is_silent_now && last_frame_was_silent)) {
            Serial.print(raw_rms_ch0);
            Serial.print(',');
            Serial.print(final_vu_db_ch0);
            Serial.print(',');
            Serial.print(pk_db_ch0);
            Serial.print('\t');
            Serial.print(raw_rms_ch1);
            Serial.print(',');
            Serial.print(final_vu_db_ch1);
            Serial.print(',');
            Serial.print(pk_db_ch1);
            Serial.print('\t');
            Serial.println(frame_work_percent);
        }

    if (display_mode_selected && display_is_on && should_update_display)
    {
      draw_audio_metrics(new_display_mode ? 0 : 20,
                         new_display_mode ? NEW_METRICS_Y : 205,
                         raw_rms_ch0, final_vu_db_ch0, pk_db_ch0,
                         raw_rms_ch1, final_vu_db_ch1, pk_db_ch1,
                         frame_work_percent);
    }

    // compute cpu usage once per second
    if (display_is_on && !(is_silent_now && last_frame_was_silent))
    {      
      static uint32_t last_report = 0;
      uint32_t now = millis();
      if (now - last_report >= 1000)
      {
        last_report = now;
        uint32_t average_frame_work_us = frame_work_total_us / 25;
        frame_work_percent = (average_frame_work_us * 100UL) / 40000UL;
        // Serial.print(average_frame_work_us);
        // Serial.print('\t');
        // Serial.print(frame_work_percent);
        // Serial.println('%');
        frame_work_total_us = 0;
      }
    // account for all loop, will show up next loop run
    frame_work_us = micros() - reading0;
    frame_work_total_us += micros() - reading0;  
    }
  }
}
