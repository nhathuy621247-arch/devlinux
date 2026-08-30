#include <string.h>
#include <assert.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ST7796_SPI";

//HARDWARE DEFINITIONS

#define LCD_HOST      SPI2_HOST     //SPI2_HOST trên ESP32-S3 chức năng tương đương VSPI trên ESP32 gốc)
#define PIN_SCK       GPIO_NUM_12
#define PIN_MOSI      GPIO_NUM_11
#define PIN_MISO      GPIO_NUM_13
#define PIN_CS        GPIO_NUM_10
#define PIN_RS        GPIO_NUM_9    // command (mức thấp) / data (mức cao)
#define PIN_RST       GPIO_NUM_14
#define PIN_BK_LIGHT  GPIO_NUM_2

#define LCD_H_RES     (480U) // landscape: chiều rộng
#define LCD_V_RES     (320U) // landscape: chiều cao
#define LCD_CLK_HZ    (20 * 1000 * 1000) // 20 MHz - trong ngưỡng ST7796U datasheet hỗ trợ, tăng dần sau khi mọi thứ đã chạy ổn định

// Mỗi pixel gửi đi 2 byte (RGB565), byte cao trước (big-endian) theo đúng cách ST7796U diễn giải dữ liệu RAMWR.
#define CHUNK_PIXELS  (4800U)
#define CHUNK_BYTES   (CHUNK_PIXELS * 2U) // 4800 pixel = 9600 byte

//ST7796U COMMANDS

#define CMD_SWRESET   (0x01U)
#define CMD_SLPOUT    (0x11U)
#define CMD_INVON     (0x21U)
#define CMD_DISPON    (0x29U)
#define CMD_CASET     (0x2AU)
#define CMD_RASET     (0x2BU)
#define CMD_RAMWR     (0x2CU)
#define CMD_MADCTL    (0x36U)
#define CMD_COLMOD    (0x3AU)

#define COLMOD_16BIT_RGB565 (0x55U) // độ sâu màu 16-bit RGB565

// MADCTL BITS

#define MADCTL_MY     (0x80U) //thứ tự địa chỉ hàng
#define MADCTL_MX     (0x40U) // thứ tự địa chỉ cột
#define MADCTL_MV     (0x20U) // hoán đổi hàng/cột -> chính bit này biến panel từ portrait sang landscape
#define MADCTL_BGR    (0x08U) // thứ tự màu: bật = BGR, tắt = RGB


#define LCD_MADCTL_VALUE (MADCTL_MV | MADCTL_MX | MADCTL_BGR)

// COLOURS (RGB565)

#define COLOUR_RED    (0xF800U)
#define COLOUR_GREEN  (0x07E0U)
#define COLOUR_BLUE   (0x001FU)
#define COLOUR_WHITE  (0xFFFFU)
#define COLOUR_BLACK  (0x0000U)

//Kích thước 3 thanh màu
//320 / 3 = 106.67 -> chia 106 / 106 / 108 để tổng đúng bằng 320, không để dư pixel nào chưa được vẽ ở đáy màn hình.
#define BAR1_HEIGHT   (106U)
#define BAR2_HEIGHT   (106U)
#define BAR3_HEIGHT   (108U) // 320 - BAR1_HEIGHT - BAR2_HEIGHT

//TIMING (đơn vị mili-giây)
//Reset phần cứng: ST7796U chỉ yêu cầu ~10ms, ở đây giữ mức 100ms/100ms (low/high) để có biên an toàn lớn, không phụ thuộc chất lượng nguồn.
#define RESET_LOW_DELAY_MS   (100U)
#define RESET_HIGH_DELAY_MS  (100U)
// Datasheet ST7796U yêu cầu tối thiểu ~115ms sau SWRESET và sau SLPOUT trước khi gửi lệnh tiếp theo; dùng 120ms để có biên an toàn.
#define CMD_DELAY_MS         (120U)
// Đợi thêm sau DISPON trước khi bật đèn nền, tránh hiện tượng nhấp nháy/artifact do panel chưa kịp ổn định dữ liệu hiển thị đầu tiên.
#define BACKLIGHT_DELAY_MS   (20U)

#define COLOR_CYCLE_DELAY_MS (1000U)
#define BARS_HOLD_DELAY_MS   (3000U)

//BIẾN TOÀN CỤC

static spi_device_handle_t lcd_spi = NULL;

//Các hàm giao tiếp SPI cấp thấp
/**
   @brief Gửi một byte lệnh tới ST7796U (LCD_RS = mức thấp).

*/
static void lcd_write_cmd(uint8_t cmd)
{
  gpio_set_level(PIN_RS, 0); //mức thấp = command

  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // khởi tạo toàn bộ struct về 0, tránh giá trị rác
  t.length    = 8; // độ dài tính bằng bit
  t.tx_buffer = &cmd;

  ESP_ERROR_CHECK(spi_device_polling_transmit(lcd_spi, &t));
}

/**
   @brief Gửi một chuỗi byte dữ liệu tới ST7796U (LCD_RS = mức cao).

   Buffer rỗng (len == 0) được trả về sớm để tránh tạo một giao dịch SPI
   không mang dữ liệu gì cả.
*/
static void lcd_write_data(const uint8_t *data, size_t len)
{
  if (len == 0) {
    return; // trường hợp biên: không có gì để gửi
  }

  gpio_set_level(PIN_RS, 1); //mức cao = data

  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length    = len * 8;
  t.tx_buffer = data;

  ESP_ERROR_CHECK(spi_device_polling_transmit(lcd_spi, &t));
}

//Các hàm cửa sổ vẽ và tô màu
/**
   @brief Thiết lập vùng cửa sổ ghi pixel bằng CASET/RASET, sau đó mở RAMWR.

   Toạ độ theo chuẩn ST7796U: mỗi giá trị 16-bit được gửi big-endian
   (byte cao trước, byte thấp sau).
*/
static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint8_t caset[4] = {
    (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFFU),
    (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFFU),
  };
  lcd_write_cmd(CMD_CASET);
  lcd_write_data(caset, sizeof(caset));

  uint8_t raset[4] = {
    (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFFU),
    (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFFU),
  };
  lcd_write_cmd(CMD_RASET);
  lcd_write_data(raset, sizeof(raset));

  lcd_write_cmd(CMD_RAMWR);
}

/**
   @brief Tô đầy một vùng chữ nhật (x, y, w, h) bằng một màu RGB565 duy nhất.

*/
static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t colour)
{
  lcd_set_window(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));

  // Byte cao trước (big-endian) cho mỗi pixel RGB565.
  uint8_t hi = (uint8_t)(colour >> 8);
  uint8_t lo = (uint8_t)(colour & 0xFFU);

  // Cấp phát buffer chunk trong vùng nhớ DMA-capable để spi_master có thể truyền trực tiếp bằng DMA.
  uint8_t *chunk_buf = (uint8_t *)heap_caps_malloc(CHUNK_BYTES, MALLOC_CAP_DMA);
  assert(chunk_buf != NULL); // dừng ngay nếu cấp phát thất bại, thay vì tiếp tục chạy với con trỏ NULL

  for (uint32_t i = 0; i < CHUNK_PIXELS; i++) {
    chunk_buf[2U * i]      = hi;
    chunk_buf[2U * i + 1U] = lo;
  }

  uint32_t total_pixels = (uint32_t)w * (uint32_t)h;
  uint32_t remaining    = total_pixels;

  while (remaining > 0U) {
    uint32_t pixels_this_chunk = (remaining > CHUNK_PIXELS) ? CHUNK_PIXELS : remaining;
    lcd_write_data(chunk_buf, (size_t)pixels_this_chunk * 2U);
    remaining -= pixels_this_chunk;
  }

  free(chunk_buf);
}

// Khởi tạo màn hình

/**
   @brief Reset phần cứng và chạy chuỗi lệnh khởi tạo tối thiểu cho ST7796U.

   Thứ tự bắt buộc: SWRESET -> đợi -> SLPOUT -> đợi -> MADCTL -> COLMOD ->
   INVON -> DISPON. Bỏ qua các khoảng đợi bắt buộc sẽ khiến panel vẫn đen dù
   lệnh vẫn "chạy được" về mặt phần mềm.
*/
static void lcd_init(void)
{
  // Cấu hình các chân điều khiển rời (không thuộc bus SPI) làm output.
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << PIN_RS) | (1ULL << PIN_RST) | (1ULL << PIN_BK_LIGHT),
    .mode         = GPIO_MODE_OUTPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  // Đèn nền tắt trong lúc khởi tạo, chỉ bật sau khi DISPON ổn định.
  ESP_ERROR_CHECK(gpio_set_level(PIN_BK_LIGHT, 0));

  // Reset phần cứng: kéo LCD_RST xuống thấp rồi thả lên cao, mỗi pha giữ đủ lâu để chắc chắn panel nhận được cạnh reset sạch.
  ESP_ERROR_CHECK(gpio_set_level(PIN_RST, 0));
  vTaskDelay(pdMS_TO_TICKS(RESET_LOW_DELAY_MS));
  ESP_ERROR_CHECK(gpio_set_level(PIN_RST, 1));
  vTaskDelay(pdMS_TO_TICKS(RESET_HIGH_DELAY_MS));

  // Software reset - đợi đúng thời gian datasheet quy định trước khi gửi lệnh kế tiếp.
  lcd_write_cmd(CMD_SWRESET);
  vTaskDelay(pdMS_TO_TICKS(CMD_DELAY_MS));

  // Thoát chế độ sleep - cũng cần một khoảng đợi bắt buộc tương tự.
  lcd_write_cmd(CMD_SLPOUT);
  vTaskDelay(pdMS_TO_TICKS(CMD_DELAY_MS));

  // Thiết lập hướng hiển thị landscape 480x320 + thứ tự màu BGR.
  lcd_write_cmd(CMD_MADCTL);
  uint8_t madctl = LCD_MADCTL_VALUE;
  lcd_write_data(&madctl, 1);

  // Độ sâu màu 16-bit RGB565.
  lcd_write_cmd(CMD_COLMOD);
  uint8_t colmod = COLMOD_16BIT_RGB565;
  lcd_write_data(&colmod, 1);

  // Panel IPS này cần bật Display Inversion, nếu không màu sẽ bị đảo ngược (đen thành trắng, trắng thành đen).
  lcd_write_cmd(CMD_INVON);

  // Bật hiển thị.
  lcd_write_cmd(CMD_DISPON);

  // Đợi thêm trước khi bật đèn nền để tránh artifact khi panel chưa kịp ổn định khung hình đầu tiên.
  vTaskDelay(pdMS_TO_TICKS(BACKLIGHT_DELAY_MS));
  ESP_ERROR_CHECK(gpio_set_level(PIN_BK_LIGHT, 1));

  ESP_LOGI(TAG, "ST7796U initialised: %ux%u landscape, RGB565", LCD_H_RES, LCD_V_RES);
}

//Main application

void app_main(void)
{
  // Khởi tạo SPI bus. max_transfer_sz phải >= CHUNK_BYTES, nếu không giao dịch chunk sẽ lỗi ở runtime chứ không phải lúc biên dịch.
  spi_bus_config_t buscfg = {
    .sclk_io_num     = PIN_SCK,
    .mosi_io_num     = PIN_MOSI,
    .miso_io_num     = PIN_MISO,
    .quadwp_io_num   = -1,
    .quadhd_io_num   = -1,
    .max_transfer_sz = CHUNK_BYTES,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // Gắn thiết bị ST7796U vào bus SPI vừa khởi tạo.
  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = LCD_CLK_HZ,
    .mode           = 0,
    .spics_io_num   = PIN_CS,
    .queue_size     = 7,
  };
  ESP_ERROR_CHECK(spi_bus_add_device(LCD_HOST, &devcfg, &lcd_spi));

  // Khởi tạo controller màn hình.
  lcd_init();

  //Vẽ ba thanh màu ngang, mỗi thanh chiếm 1/3 chiều cao, để chụp ảnh xác nhận kết quả.
  lcd_fill_rect(0, 0, LCD_H_RES, BAR1_HEIGHT, COLOUR_RED);
  lcd_fill_rect(0, BAR1_HEIGHT, LCD_H_RES, BAR2_HEIGHT, COLOUR_GREEN);
  lcd_fill_rect(0, BAR1_HEIGHT + BAR2_HEIGHT, LCD_H_RES, BAR3_HEIGHT, COLOUR_BLUE);

  vTaskDelay(pdMS_TO_TICKS(BARS_HOLD_DELAY_MS));

  // Vòng lặp vô hạn: đổi màu toàn màn hình mỗi giây, theo thứ tự đỏ -> xanh lá -> xanh dương -> trắng -> đen -> quay lại đỏ.
  static const uint16_t colour_cycle[] = {
    COLOUR_RED, COLOUR_GREEN, COLOUR_BLUE, COLOUR_WHITE, COLOUR_BLACK,
  };
  const size_t colour_count = sizeof(colour_cycle) / sizeof(colour_cycle[0]);
  size_t idx = 0;

  for (;;) {
    lcd_fill_rect(0, 0, LCD_H_RES, LCD_V_RES, colour_cycle[idx]);
    idx = (idx + 1U) % colour_count; //quay vòng về đầu sau màu cuối
    vTaskDelay(pdMS_TO_TICKS(COLOR_CYCLE_DELAY_MS));
  }
}
