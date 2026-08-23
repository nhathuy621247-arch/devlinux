/* ============================================================================
   Session 04 - Exercise 1 : Bo dem 7 doan dieu khien bang nut nhan, ban ngat
   Board muc tieu : ESP32-S3 DevKitC-1
   Loai man hinh : COMMON CATHODE (doan SANG = chan duoc keo len muc CAO)

   File nay co hai lop song song, co chu dich:

     MAN HINH (doan a..g) -> muc thanh ghi, giu nguyen tu Session 03.
                              IO_MUX + GPIO matrix + GPIO_OUT_W1TS/W1TC.

     NUT NHAN (GPIO14)    -> muc driver (driver/gpio.h), phat hien bang ngat.
                              gpio_config() + gpio_install_isr_service() +
                              gpio_isr_handler_add(), GPIO_INTR_ANYEDGE.

   Chi co duong phat hien nut nhan la thay doi so voi Session 03. May trang
   thai giai ma cu chi (click / double-click / long-press tu dong lap) gio
   duoc dieu khien boi cac su kien trong hang doi + mot timeout tinh dong,
   thay vi vong lap poll voi chu ky co dinh.
   ============================================================================
*/

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_err.h"

static const char *TAG = "BTN";

/* ============================================================================
   SECTION: Loai man hinh
   ============================================================================ */
#define DISPLAY_COMMON_ANODE   (0)   /* 0 = common cathode, 1 = common anode */

/* ============================================================================
   SECTION: Cac chan  (giong het cach noi day cua Session 03 - khong noi lai)
   ============================================================================ */
#define SEG_A_PIN   (4U)
#define SEG_B_PIN   (5U)
#define SEG_C_PIN   (6U)
#define SEG_D_PIN   (7U)
#define SEG_E_PIN   (15U)
#define SEG_F_PIN   (16U)
#define SEG_G_PIN   (17U)

#define NUM_SEGMENTS (7U)
static const uint32_t SEG_PINS[NUM_SEGMENTS] = {
  SEG_A_PIN, SEG_B_PIN, SEG_C_PIN, SEG_D_PIN, SEG_E_PIN, SEG_F_PIN, SEG_G_PIN
};

#define BTN_PIN      GPIO_NUM_14   /* kieu enum cua driver, dung cho gpio_config()/API ngat */
#define BTN_PIN_NUM  (14U)         /* so chan thuan, dung de dung bit mask                  */

/* bit0..bit6 = doan a..g, bang nay danh cho man hinh COMMON CATHODE */
static const uint8_t SEGMENT_MAP[10] = {
  0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U, /* 0 1 2 3 4 */
  0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU, /* 5 6 7 8 9 */
};

/* ============================================================================
   SECTION: Cac hang so thoi gian
   ============================================================================ */
#define DEBOUNCE_MS       (25U)   /* thoi gian cho tin hieu on dinh sau khi nay hoi tiep xuc */
#define DOUBLE_CLICK_MS   (350U)  /* khoang cach toi da giua hai lan tha de tinh la double   */
#define LONG_PRESS_MS     (800U)  /* thoi gian giu truoc khi bat dau tu dong lap             */
#define REPEAT_PERIOD_MS  (500U)  /* chu ky tu dong lap khi con giu nut (theo de bai)        */

/* ============================================================================
   SECTION: Driver man hinh muc thanh ghi  (chep tu Session 03, khong doi)
   ============================================================================ */
#define GPIO_BASE     (0x60004000UL)
#define IO_MUX_BASE   (0x60009000UL)

#define GPIO_OUT_REG          (GPIO_BASE + 0x0004UL)
#define GPIO_OUT_W1TS_REG     (GPIO_BASE + 0x0008UL)  /* ghi 1 vao bit nao -> bit do len 1, tuc thi   */
#define GPIO_OUT_W1TC_REG     (GPIO_BASE + 0x000CUL)  /* ghi 1 vao bit nao -> bit do ve 0, tuc thi     */
#define GPIO_ENABLE_W1TS_REG  (GPIO_BASE + 0x0024UL)  /* bat driver ngo ra cho mot chan                */
#define GPIO_ENABLE_W1TC_REG  (GPIO_BASE + 0x0028UL)  /* tat driver ngo ra cho mot chan                */

/* GPIO matrix: moi chan co mot thanh ghi "chon tin hieu ngo ra" rieng, cach nhau 4 byte */
#define GPIO_FUNC_OUT_SEL(pin)   (*(volatile uint32_t *)(GPIO_BASE + 0x0554UL + 4UL * (pin)))
#define GPIO_MATRIX_SIG_GPIO_OUT (256UL)       /* SIG_GPIO_OUT_IDX: chan noi thang toi
                                                   GPIO_OUT_REG, bo qua moi ngoai vi          */
#define GPIO_MATRIX_OE_SOFTWARE  (1UL << 10U)  /* output-enable lay tu GPIO_ENABLE_REG,
                                                   khong lay tu tin hieu OE cua ngoai vi nao   */

#define IOMUX(offset) (IO_MUX_BASE + (offset))
static const uint32_t IO_MUX_OF_PIN[18] = {
  [4]  = IOMUX(0x14UL),
  [5]  = IOMUX(0x18UL),
  [6]  = IOMUX(0x1CUL),
  [7]  = IOMUX(0x20UL),
  [15] = IOMUX(0x40UL),
  [16] = IOMUX(0x44UL),
  [17] = IOMUX(0x48UL),
};

/* Cau truc bit cua IO_MUX_GPIOn_REG - giong nhau tren moi chan (TRM "IO MUX and GPIO Matrix") */
#define IO_MUX_MCU_SEL_SHIFT   (12U)
#define IO_MUX_MCU_SEL_MASK    (0x7U << IO_MUX_MCU_SEL_SHIFT)
#define IO_MUX_MCU_SEL_GPIO    (1U)            /* chuc nang 1 = GPIO thuong tren moi chan dung o day */
#define IO_MUX_FUN_DRV_SHIFT   (10U)
#define IO_MUX_FUN_DRV_MASK    (0x3U << IO_MUX_FUN_DRV_SHIFT)
#define IO_MUX_FUN_DRV_DEFAULT (2U)            /* do manh dong ra mac dinh (trung binh)              */
#define IO_MUX_FUN_IE_BIT      (1U << 9U)      /* bat bo dem ngo vao                                 */
#define IO_MUX_FUN_WPU_BIT     (1U << 8U)      /* bat pull-up noi                                    */
#define IO_MUX_FUN_WPD_BIT     (1U << 7U)      /* bat pull-down noi                                  */

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* Cau hinh mot chan thanh ngo ra GPIO thuong, muc thap luc khoi dong. */
static void seg7_pin_init(uint32_t pin)
{
  uint32_t iomux_addr = IO_MUX_OF_PIN[pin];

  uint32_t v = REG32(iomux_addr);
  v &= ~IO_MUX_MCU_SEL_MASK;
  v |= (IO_MUX_MCU_SEL_GPIO << IO_MUX_MCU_SEL_SHIFT);
  v &= ~IO_MUX_FUN_DRV_MASK;
  v |= (IO_MUX_FUN_DRV_DEFAULT << IO_MUX_FUN_DRV_SHIFT);
  v &= ~IO_MUX_FUN_IE_BIT;    /* la ngo ra, khong can doc lai */
  v &= ~IO_MUX_FUN_WPU_BIT;
  v &= ~IO_MUX_FUN_WPD_BIT;
  REG32(iomux_addr) = v;

  /* GPIO matrix: noi chan thang tu GPIO_OUT_REG, OE lay tu phan mem */
  GPIO_FUNC_OUT_SEL(pin) = GPIO_MATRIX_SIG_GPIO_OUT | GPIO_MATRIX_OE_SOFTWARE;

  REG32(GPIO_ENABLE_W1TS_REG) = (1UL << pin);  /* bat driver ngo ra    */
  REG32(GPIO_OUT_W1TC_REG)    = (1UL << pin);  /* khoi dong muc thap (doan tat) */
}

static void seg7_init(void)
{
  for (uint32_t i = 0U; i < NUM_SEGMENTS; i++) {
    seg7_pin_init(SEG_PINS[i]);
  }
}

/* Cap nhat ca bay doan chi bang mot lan ghi W1TS va mot lan ghi W1TC, nen ca
   chu so doi cung mot luc thay vi tung doan mot. */
static void seg7_show_digit(uint8_t digit)
{
  uint8_t pattern = SEGMENT_MAP[digit % 10U];

#if DISPLAY_COMMON_ANODE
  pattern = (uint8_t)(~pattern);   /* dao bit duy nhat, dat ten ro rang, danh cho common anode */
#endif

  uint32_t set_mask = 0U;
  uint32_t clr_mask = 0U;

  for (uint32_t i = 0U; i < NUM_SEGMENTS; i++) {
    uint32_t bit = (1UL << SEG_PINS[i]);
    if ((pattern >> i) & 0x1U) {
      set_mask |= bit;
    } else {
      clr_mask |= bit;
    }
  }

  REG32(GPIO_OUT_W1TS_REG) = set_mask;
  REG32(GPIO_OUT_W1TC_REG) = clr_mask;
}

/* ============================================================================
   SECTION: Trang thai bo dem
   ============================================================================ */
static volatile uint8_t counter_value = 0U;

static inline void digit_step(int delta)
{
  int v = (int)counter_value + delta;
  v %= 10;
  if (v < 0) {
    v += 10;
  }
  counter_value = (uint8_t)v;
}

/* ============================================================================
   SECTION: Nut nhan - muc driver GPIO + ngat
   ============================================================================ */
typedef struct {
  int64_t timestamp_us;  /* esp_timer_get_time() tai thoi diem xay ra canh */
  bool    is_press;      /* true = canh xuong (nhan), false = canh len (tha) */
} btn_event_t;

static QueueHandle_t btn_queue = NULL;

/* Boi canh ISR: esp_timer_get_time() va gpio_get_level() deu nam trong IRAM
   va an toan de goi tu ngat. Khong lam gi khac o day: khong log, khong delay,
   khong tinh toan chu so, khong ghi thanh ghi man hinh. Su kien chi duoc
   lay dau thoi gian va chuyen giao, do la toan bo cong viec cua ISR nay. */
static void IRAM_ATTR button_isr(void *arg)
{
  (void)arg;

  btn_event_t ev;
  ev.timestamp_us = esp_timer_get_time();
  ev.is_press = (gpio_get_level(BTN_PIN) == 0);  /* co pull-up: nhan thi doc duoc 0 */

  BaseType_t hp_task_woken = pdFALSE;
  xQueueSendFromISR(btn_queue, &ev, &hp_task_woken);
  portYIELD_FROM_ISR(hp_task_woken);
}

/* Chi cau hinh chan. Goi truoc khi hang doi/ISR service ton tai cung an toan,
   vi GPIO_INTR_ANYEDGE chua the thuc su goi toi button_isr() cho den khi
   gpio_install_isr_service() + gpio_isr_handler_add() chay ve sau. */
static void button_pin_config(void)
{
  gpio_config_t cfg = {
    .pin_bit_mask = (1ULL << BTN_PIN_NUM),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_ENABLE,     /* nhan thi doc duoc 0, dung theo cach noi day */
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_ANYEDGE,      /* can ca canh nhan lan canh tha */
  };
  ESP_ERROR_CHECK(gpio_config(&cfg));
}

/* Cai dat ISR service va dang ky handler. Chi duoc goi sau khi btn_queue da
   ton tai va task tieu thu da chay, neu khong mot canh co the xay ra ma
   khong co ai dang lang nghe no. */
static void button_isr_install(void)
{
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_PIN, button_isr, NULL));
}

/* ============================================================================
   SECTION: Task giai ma cu chi

   Trang thai chi ton tai trong task nay (ISR khong bao gio dung toi):
     btn_down               - trang thai nut nhan da qua debounce
     last_edge_us           - thoi diem canh *duoc chap nhan* gan nhat, dung de debounce
     press_start_us         - thoi diem lan nhan hien tai bat dau
     long_press_active      - true khi da bat dau tu dong lap cho lan nhan nay
     next_repeat_us         - thoi diem cua tick tu dong lap ke tiep
     click_pending          - vua co mot click ngan, co the con tro thanh double
     click_pending_since_us - thoi diem tha cua cai click dang cho do
     click_is_double        - chot tai thoi diem nhan: lan nhan nay co hoan tat double khong?

   Task block tren hang doi voi mot timeout duoc tinh lai moi vong lap, dua
   tren bat ky deadline nao dang hoat dong (nguong long-press, tick lap lai,
   hay cua so double-click). Khi khong co gi dang cho, no block vo han
   (portMAX_DELAY): khong ton CPU khi nut khong bi dung toi.
   ============================================================================ */

static bool    btn_down               = false;
static int64_t last_edge_us           = 0;
static int64_t press_start_us         = 0;
static bool    long_press_active      = false;
static int64_t next_repeat_us         = 0;
static bool    click_pending          = false;
static int64_t click_pending_since_us = 0;
static bool    click_is_double        = false;

static TickType_t compute_wait_ticks(void)
{
  int64_t now_us = esp_timer_get_time();
  int64_t deadline_us = -1;   /* -1 = khong co deadline, block vo han */

  if (btn_down && !long_press_active) {
    deadline_us = press_start_us + (int64_t)LONG_PRESS_MS * 1000;
  } else if (btn_down && long_press_active) {
    deadline_us = next_repeat_us;
  } else if (click_pending) {
    deadline_us = click_pending_since_us + (int64_t)DOUBLE_CLICK_MS * 1000;
  }

  if (deadline_us < 0) {
    return portMAX_DELAY;
  }

  int64_t remaining_us = deadline_us - now_us;
  if (remaining_us <= 0) {
    return 0;   /* deadline da qua - tra ve ngay lap tuc */
  }
  /* lam tron len den mili giay nguyen de khong bao gio thuc day som */
  return pdMS_TO_TICKS((uint32_t)((remaining_us + 999) / 1000));
}

static void gesture_task(void *arg)
{
  (void)arg;
  uint8_t last_shown = counter_value;

  for (;;) {
    TickType_t wait_ticks = compute_wait_ticks();

    btn_event_t ev;
    BaseType_t got_event = xQueueReceive(btn_queue, &ev, wait_ticks);
    int64_t now_us = esp_timer_get_time();

    if (got_event == pdTRUE) {
      /* Debounce: mot canh den som hon DEBOUNCE_MS sau canh *da duoc
         chap nhan* gan nhat la nay hoi tiep xuc, khong phai canh that -
         bo qua no. */
      if ((ev.timestamp_us - last_edge_us) < (int64_t)DEBOUNCE_MS * 1000) {
        continue;
      }
      /* Phong ngua truong hop bao trung lap trang thai dang co (co the
         xay ra neu mot canh nay hoi lot qua kiem tra o tren). */
      if (ev.is_press == btn_down) {
        continue;
      }

      last_edge_us = ev.timestamp_us;
      btn_down = ev.is_press;

      if (btn_down) {
        /* ---- NHAN ---- */
        press_start_us = ev.timestamp_us;
        long_press_active = false;
        click_is_double = click_pending &&
                          ((ev.timestamp_us - click_pending_since_us) <= (int64_t)DOUBLE_CLICK_MS * 1000);
        click_pending = false;   /* da duoc tieu thu du theo huong nao */
      } else {
        /* ---- THA ---- */
        if (long_press_active) {
          long_press_active = false;
          ESP_LOGI(TAG, "LONG end");
          /* khong tinh click / double - long press ket thuc trong lang le */
        } else if (click_is_double) {
          digit_step(-1);
          click_is_double = false;
          ESP_LOGI(TAG, "DOUBLE     -> %u", counter_value);
        } else {
          /* Chua the chot ngay: mot lan click thu hai co the con
             den trong DOUBLE_CLICK_MS va bien no thanh mot lan giam. */
          click_pending = true;
          click_pending_since_us = ev.timestamp_us;
        }
      }
    } else {
      /* ---- Timeout: mot deadline dang cho da toi ---- */
      if (btn_down && !long_press_active &&
          (now_us - press_start_us) >= (int64_t)LONG_PRESS_MS * 1000) {
        long_press_active = true;
        next_repeat_us = now_us + (int64_t)REPEAT_PERIOD_MS * 1000;
        digit_step(+1);
        ESP_LOGI(TAG, "LONG start -> %u", counter_value);
      } else if (btn_down && long_press_active &&
                 now_us >= next_repeat_us) {
        next_repeat_us += (int64_t)REPEAT_PERIOD_MS * 1000;
        digit_step(+1);
        ESP_LOGI(TAG, "LONG rep   -> %u", counter_value);
      } else if (click_pending &&
                 (now_us - click_pending_since_us) >= (int64_t)DOUBLE_CLICK_MS * 1000) {
        click_pending = false;
        digit_step(+1);
        ESP_LOGI(TAG, "CLICK      -> %u", counter_value);
      }
    }

    if (counter_value != last_shown) {
      last_shown = counter_value;
      seg7_show_digit(last_shown);
    }
  }
}

/* ============================================================================
   SECTION: app_main

   Thu tu rat quan trong: man hinh phai hoat dong va hang doi + task giai ma
   cu chi phai ton tai *truoc khi* ISR duoc cai dat, neu khong mot canh co the
   xay ra ma khong co noi nao de gui su kien toi.
   ============================================================================ */
void app_main(void)
{
  seg7_init();
  seg7_show_digit(0U);

  button_pin_config();

  btn_queue = xQueueCreate(8, sizeof(btn_event_t));
  configASSERT(btn_queue != NULL);

  BaseType_t task_ok = xTaskCreate(gesture_task, "gesture_task", 4096, NULL, 10, NULL);
  configASSERT(task_ok == pdPASS);

  button_isr_install();   /* chi tu bay gio button_isr() moi thuc su co the chay */

  ESP_LOGI(TAG, "san sang, counter = 0");
}
