
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "led_strip.h"

static const char *TAG = "UART_CONSOLE";

#define UART_PORT_NUM   UART_NUM_0
#define UART_TX_PIN     GPIO_NUM_43
#define UART_RX_PIN     GPIO_NUM_44
#define UART_BAUD_RATE  115200UL
#define UART_BUF_SIZE   1024U   // kích thước ring buffer RX/TX, đơn vị byte
#define UART_QUEUE_SIZE 10U     // độ sâu hàng đợi sự kiện của UART driver

#define CMD_BUF_SIZE    64U     // buffer chứa một dòng lệnh, tính cả ký tự NUL

#define RGB_LED_PIN         GPIO_NUM_38
#define LED_STRIP_MAX_LEDS  1U
#define LED_RMT_RES_HZ      (10 * 1000 * 1000) // 10 MHz, đủ độ phân giải cho timing bit của WS2812 (~800 kHz bit rate)

// Các lệnh console được hỗ trợ - không dùng magic string trong bộ phân tích.
#define CMD_LED_ON   "LED_ON"
#define CMD_LED_OFF  "LED_OFF"
#define CMD_RED      "RED"
#define CMD_GREEN    "GREEN"
#define CMD_BLUE     "BLUE"

// Giá trị màu ở độ sáng tối đa, dùng cùng led_strip_set_pixel().
#define COLOR_FULL   255U
#define COLOR_OFF    0U

#define ASCII_BACKSPACE 0x08
#define ASCII_DEL       0x7F
#define ASCII_CR        '\r'
#define ASCII_LF        '\n'

#define UART_EVENT_TASK_STACK_SIZE 4096U
#define UART_EVENT_TASK_PRIORITY   5U
#define UART_EVENT_TASK_CORE_ID    1

//Biến toàn cục

static QueueHandle_t   uart_queue     = NULL;
static led_strip_handle_t led_strip   = NULL;

//Bộ xử lý lệnh (command header)

/**
   @brief Phân tích một dòng lệnh đã nhận đầy đủ và điều khiển LED tương ứng.

   Chỉ thay đổi trạng thái LED khi nhận được lệnh hợp lệ. Không bao giờ block,
   cấp phát bộ nhớ, hay gọi các hàm họ printf - an toàn khi gọi từ task xử lý
   sự kiện UART.

   @param cmd Chuỗi lệnh đã kết thúc bằng NUL (khoảng trắng đầu/cuối đã được
              loại bỏ bởi hàm gọi).
*/
static void handle_command(const char *cmd)
{
  ESP_LOGI(TAG, "Received command: \"%s\"", cmd);

  if (strcmp(cmd, CMD_LED_ON) == 0) {
    led_strip_set_pixel(led_strip, 0, COLOR_FULL, COLOR_FULL, COLOR_FULL);
    led_strip_refresh(led_strip);
    ESP_LOGI(TAG, "LED -> ON (white)");
  } else if (strcmp(cmd, CMD_LED_OFF) == 0) {
    led_strip_clear(led_strip);
    ESP_LOGI(TAG, "LED -> OFF");
  } else if (strcmp(cmd, CMD_RED) == 0) {
    led_strip_set_pixel(led_strip, 0, COLOR_FULL, COLOR_OFF, COLOR_OFF);
    led_strip_refresh(led_strip);
    ESP_LOGI(TAG, "LED -> RED");
  } else if (strcmp(cmd, CMD_GREEN) == 0) {
    led_strip_set_pixel(led_strip, 0, COLOR_OFF, COLOR_FULL, COLOR_OFF);
    led_strip_refresh(led_strip);
    ESP_LOGI(TAG, "LED -> GREEN");
  } else if (strcmp(cmd, CMD_BLUE) == 0) {
    led_strip_set_pixel(led_strip, 0, COLOR_OFF, COLOR_OFF, COLOR_FULL);
    led_strip_refresh(led_strip);
    ESP_LOGI(TAG, "LED -> BLUE");
  } else {
    // Lệnh không xác định: cảnh báo, giữ nguyên trạng thái LED, tiếp tục chạy.
    ESP_LOGW(TAG, "Unknown command: \"%s\"", cmd);
  }
}

//Task xử lý sự kiện UART

/**
   @brief Task riêng, block trên hàng đợi sự kiện của UART driver và triển
          khai console kiểu interrupt-driven (echo, chỉnh sửa dòng lệnh,
          phân phối lệnh, phục hồi khi tràn buffer).
*/
static void uart_event_task(void *pvParameters)
{
  uart_event_t event;
  uint8_t *dtmp = (uint8_t *)malloc(UART_BUF_SIZE);
  char cmd_buf[CMD_BUF_SIZE];
  size_t cmd_len = 0;

  if (dtmp == NULL) {
    ESP_LOGE(TAG, "Failed to allocate UART RX staging buffer");
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "Console ready on UART0, 115200-8-N-1");

  for (;;) {
    // Block vô hạn cho tới khi ISR đẩy một sự kiện vào hàng đợi - không polling.
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    switch (event.type) {
      case UART_DATA: {
          int len = uart_read_bytes(UART_PORT_NUM, dtmp, event.size, portMAX_DELAY);
          if (len < 0) {
            break;
          }

          for (int i = 0; i < len; i++) {
            char c = (char)dtmp[i];

            if (c == ASCII_CR || c == ASCII_LF) {
              // Xử lý phím Enter: kết thúc dòng và phân phối lệnh.
              cmd_buf[cmd_len] = '\0';
              uart_write_bytes(UART_PORT_NUM, "\r\n", 2);
              if (cmd_len > 0) {
                handle_command(cmd_buf);
              }
              cmd_len = 0;
            } else if (c == ASCII_BACKSPACE || c == ASCII_DEL) {
              // Xử lý Backspace hoặc Delete: xóa ký tự cuối, echo để xóa trên màn hình.
              if (cmd_len > 0) {
                cmd_len--;
                uart_write_bytes(UART_PORT_NUM, "\b \b", 3);
              }
            } else if (c >= 0x20 && c < 0x7F) {
              // Xử lý ký tự in được (printable): thêm vào buffer + echo ngay lập tức.
              if (cmd_len < (CMD_BUF_SIZE - 1U)) {
                cmd_buf[cmd_len] = c;
                cmd_len++;
                uart_write_bytes(UART_PORT_NUM, &c, 1);
              }

            }
            // Các ký tự điều khiển khác được bỏ qua.
          }
          break;
        }

      case UART_FIFO_OVF:
      case UART_BUFFER_FULL:
        // Phục hồi một cách xác định: bỏ toàn bộ dữ liệu đang dở và reset trạng thái dòng lệnh thay vì âm thầm bỏ qua sự kiện này.
        uart_flush_input(UART_PORT_NUM);
        xQueueReset(uart_queue);
        cmd_len = 0;
        ESP_LOGW(TAG, "UART overflow, input flushed");
        break;

      default:
        ESP_LOGI(TAG, "Unhandled UART event type: %d", event.type);
        break;
    }
  }

  free(dtmp);
  vTaskDelete(NULL);
}

//Main application

void app_main(void)
{
  // Khởi tạo WS2812 gắn sẵn trên board và đảm bảo nó tắt lúc khởi động.
  led_strip_config_t strip_config = {
    .strip_gpio_num = RGB_LED_PIN,
    .max_leds       = LED_STRIP_MAX_LEDS,
    .led_model      = LED_MODEL_WS2812,
    .led_pixel_format = LED_PIXEL_FORMAT_GRB, // thứ tự byte của WS2812
    .flags = {
      .invert_out = false,
    },
  };
  led_strip_rmt_config_t rmt_config = {
    .clk_src        = RMT_CLK_SRC_DEFAULT,
    .resolution_hz  = LED_RMT_RES_HZ, // tick 10 MHz, đủ phân giải timing bit của WS2812
    .flags = {
      .with_dma = false, //chỉ một pixel - không cần DMA
    },
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  ESP_ERROR_CHECK(led_strip_clear(led_strip));

  // Khởi động UART0 trên các chân nối với chip cầu CP2102N.
  uart_config_t uart_config = {
    .baud_rate  = UART_BAUD_RATE,
    .data_bits  = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE,
                                      UART_QUEUE_SIZE, &uart_queue, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // Giao vòng lặp console interrupt-driven cho một task riêng, pin vào core.
  xTaskCreatePinnedToCore(uart_event_task,
                          "uart_event_task",
                          UART_EVENT_TASK_STACK_SIZE,
                          NULL,
                          UART_EVENT_TASK_PRIORITY,
                          NULL,
                          UART_EVENT_TASK_CORE_ID);
}
