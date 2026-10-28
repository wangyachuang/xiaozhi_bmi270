#include "wifi_board.h"
#include "application.h"
#include "echoear_audio_codec.h"
#include "display/lcd_display.h"
#include "display/emote_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "backlight.h"

#include <wifi_station.h>
#include <esp_log.h>

#include <driver/i2c_master.h>
#include <driver/i2c.h>
#include "i2c_device.h"
#include <esp_afe_sr_models.h>
#include <esp_afe_doa.h>
#include <esp_timer.h>
#include <math.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_st77916.h>
#include "esp_lcd_touch_cst816s.h"
#include "touch.h"
#include "bmi270_imu.h"

#include "driver/temperature_sensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define TAG "EchoEar"
static volatile int g_doa_last_angle = 0;
static volatile int64_t g_doa_last_ms = 0;


temperature_sensor_handle_t temp_sensor = NULL;
static const st77916_lcd_init_cmd_t vendor_specific_init_yysj[] = {
    {0xF0, (uint8_t []){0x28}, 1, 0},
    {0xF2, (uint8_t []){0x28}, 1, 0},
    {0x73, (uint8_t []){0xF0}, 1, 0},
    {0x7C, (uint8_t []){0xD1}, 1, 0},
    {0x83, (uint8_t []){0xE0}, 1, 0},
    {0x84, (uint8_t []){0x61}, 1, 0},
    {0xF2, (uint8_t []){0x82}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x01}, 1, 0},
    {0xF1, (uint8_t []){0x01}, 1, 0},
    {0xB0, (uint8_t []){0x56}, 1, 0},
    {0xB1, (uint8_t []){0x4D}, 1, 0},
    {0xB2, (uint8_t []){0x24}, 1, 0},
    {0xB4, (uint8_t []){0x87}, 1, 0},
    {0xB5, (uint8_t []){0x44}, 1, 0},
    {0xB6, (uint8_t []){0x8B}, 1, 0},
    {0xB7, (uint8_t []){0x40}, 1, 0},
    {0xB8, (uint8_t []){0x86}, 1, 0},
    {0xBA, (uint8_t []){0x00}, 1, 0},
    {0xBB, (uint8_t []){0x08}, 1, 0},
    {0xBC, (uint8_t []){0x08}, 1, 0},
    {0xBD, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x80}, 1, 0},
    {0xC1, (uint8_t []){0x10}, 1, 0},
    {0xC2, (uint8_t []){0x37}, 1, 0},
    {0xC3, (uint8_t []){0x80}, 1, 0},
    {0xC4, (uint8_t []){0x10}, 1, 0},
    {0xC5, (uint8_t []){0x37}, 1, 0},
    {0xC6, (uint8_t []){0xA9}, 1, 0},
    {0xC7, (uint8_t []){0x41}, 1, 0},
    {0xC8, (uint8_t []){0x01}, 1, 0},
    {0xC9, (uint8_t []){0xA9}, 1, 0},
    {0xCA, (uint8_t []){0x41}, 1, 0},
    {0xCB, (uint8_t []){0x01}, 1, 0},
    {0xD0, (uint8_t []){0x91}, 1, 0},
    {0xD1, (uint8_t []){0x68}, 1, 0},
    {0xD2, (uint8_t []){0x68}, 1, 0},
    {0xF5, (uint8_t []){0x00, 0xA5}, 2, 0},
    {0xDD, (uint8_t []){0x4F}, 1, 0},
    {0xDE, (uint8_t []){0x4F}, 1, 0},
    {0xF1, (uint8_t []){0x10}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x02}, 1, 0},
    {0xE0, (uint8_t []){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
    {0xE1, (uint8_t []){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
    {0xF0, (uint8_t []){0x10}, 1, 0},
    {0xF3, (uint8_t []){0x10}, 1, 0},
    {0xE0, (uint8_t []){0x07}, 1, 0},
    {0xE1, (uint8_t []){0x00}, 1, 0},
    {0xE2, (uint8_t []){0x00}, 1, 0},
    {0xE3, (uint8_t []){0x00}, 1, 0},
    {0xE4, (uint8_t []){0xE0}, 1, 0},
    {0xE5, (uint8_t []){0x06}, 1, 0},
    {0xE6, (uint8_t []){0x21}, 1, 0},
    {0xE7, (uint8_t []){0x01}, 1, 0},
    {0xE8, (uint8_t []){0x05}, 1, 0},
    {0xE9, (uint8_t []){0x02}, 1, 0},
    {0xEA, (uint8_t []){0xDA}, 1, 0},
    {0xEB, (uint8_t []){0x00}, 1, 0},
    {0xEC, (uint8_t []){0x00}, 1, 0},
    {0xED, (uint8_t []){0x0F}, 1, 0},
    {0xEE, (uint8_t []){0x00}, 1, 0},
    {0xEF, (uint8_t []){0x00}, 1, 0},
    {0xF8, (uint8_t []){0x00}, 1, 0},
    {0xF9, (uint8_t []){0x00}, 1, 0},
    {0xFA, (uint8_t []){0x00}, 1, 0},
    {0xFB, (uint8_t []){0x00}, 1, 0},
    {0xFC, (uint8_t []){0x00}, 1, 0},
    {0xFD, (uint8_t []){0x00}, 1, 0},
    {0xFE, (uint8_t []){0x00}, 1, 0},
    {0xFF, (uint8_t []){0x00}, 1, 0},
    {0x60, (uint8_t []){0x40}, 1, 0},
    {0x61, (uint8_t []){0x04}, 1, 0},
    {0x62, (uint8_t []){0x00}, 1, 0},
    {0x63, (uint8_t []){0x42}, 1, 0},
    {0x64, (uint8_t []){0xD9}, 1, 0},
    {0x65, (uint8_t []){0x00}, 1, 0},
    {0x66, (uint8_t []){0x00}, 1, 0},
    {0x67, (uint8_t []){0x00}, 1, 0},
    {0x68, (uint8_t []){0x00}, 1, 0},
    {0x69, (uint8_t []){0x00}, 1, 0},
    {0x6A, (uint8_t []){0x00}, 1, 0},
    {0x6B, (uint8_t []){0x00}, 1, 0},
    {0x70, (uint8_t []){0x40}, 1, 0},
    {0x71, (uint8_t []){0x03}, 1, 0},
    {0x72, (uint8_t []){0x00}, 1, 0},
    {0x73, (uint8_t []){0x42}, 1, 0},
    {0x74, (uint8_t []){0xD8}, 1, 0},
    {0x75, (uint8_t []){0x00}, 1, 0},
    {0x76, (uint8_t []){0x00}, 1, 0},
    {0x77, (uint8_t []){0x00}, 1, 0},
    {0x78, (uint8_t []){0x00}, 1, 0},
    {0x79, (uint8_t []){0x00}, 1, 0},
    {0x7A, (uint8_t []){0x00}, 1, 0},
    {0x7B, (uint8_t []){0x00}, 1, 0},
    {0x80, (uint8_t []){0x48}, 1, 0},
    {0x81, (uint8_t []){0x00}, 1, 0},
    {0x82, (uint8_t []){0x06}, 1, 0},
    {0x83, (uint8_t []){0x02}, 1, 0},
    {0x84, (uint8_t []){0xD6}, 1, 0},
    {0x85, (uint8_t []){0x04}, 1, 0},
    {0x86, (uint8_t []){0x00}, 1, 0},
    {0x87, (uint8_t []){0x00}, 1, 0},
    {0x88, (uint8_t []){0x48}, 1, 0},
    {0x89, (uint8_t []){0x00}, 1, 0},
    {0x8A, (uint8_t []){0x08}, 1, 0},
    {0x8B, (uint8_t []){0x02}, 1, 0},
    {0x8C, (uint8_t []){0xD8}, 1, 0},
    {0x8D, (uint8_t []){0x04}, 1, 0},
    {0x8E, (uint8_t []){0x00}, 1, 0},
    {0x8F, (uint8_t []){0x00}, 1, 0},
    {0x90, (uint8_t []){0x48}, 1, 0},
    {0x91, (uint8_t []){0x00}, 1, 0},
    {0x92, (uint8_t []){0x0A}, 1, 0},
    {0x93, (uint8_t []){0x02}, 1, 0},
    {0x94, (uint8_t []){0xDA}, 1, 0},
    {0x95, (uint8_t []){0x04}, 1, 0},
    {0x96, (uint8_t []){0x00}, 1, 0},
    {0x97, (uint8_t []){0x00}, 1, 0},
    {0x98, (uint8_t []){0x48}, 1, 0},
    {0x99, (uint8_t []){0x00}, 1, 0},
    {0x9A, (uint8_t []){0x0C}, 1, 0},
    {0x9B, (uint8_t []){0x02}, 1, 0},
    {0x9C, (uint8_t []){0xDC}, 1, 0},
    {0x9D, (uint8_t []){0x04}, 1, 0},
    {0x9E, (uint8_t []){0x00}, 1, 0},
    {0x9F, (uint8_t []){0x00}, 1, 0},
    {0xA0, (uint8_t []){0x48}, 1, 0},
    {0xA1, (uint8_t []){0x00}, 1, 0},
    {0xA2, (uint8_t []){0x05}, 1, 0},
    {0xA3, (uint8_t []){0x02}, 1, 0},
    {0xA4, (uint8_t []){0xD5}, 1, 0},
    {0xA5, (uint8_t []){0x04}, 1, 0},
    {0xA6, (uint8_t []){0x00}, 1, 0},
    {0xA7, (uint8_t []){0x00}, 1, 0},
    {0xA8, (uint8_t []){0x48}, 1, 0},
    {0xA9, (uint8_t []){0x00}, 1, 0},
    {0xAA, (uint8_t []){0x07}, 1, 0},
    {0xAB, (uint8_t []){0x02}, 1, 0},
    {0xAC, (uint8_t []){0xD7}, 1, 0},
    {0xAD, (uint8_t []){0x04}, 1, 0},
    {0xAE, (uint8_t []){0x00}, 1, 0},
    {0xAF, (uint8_t []){0x00}, 1, 0},
    {0xB0, (uint8_t []){0x48}, 1, 0},
    {0xB1, (uint8_t []){0x00}, 1, 0},
    {0xB2, (uint8_t []){0x09}, 1, 0},
    {0xB3, (uint8_t []){0x02}, 1, 0},
    {0xB4, (uint8_t []){0xD9}, 1, 0},
    {0xB5, (uint8_t []){0x04}, 1, 0},
    {0xB6, (uint8_t []){0x00}, 1, 0},
    {0xB7, (uint8_t []){0x00}, 1, 0},
    {0xB8, (uint8_t []){0x48}, 1, 0},
    {0xB9, (uint8_t []){0x00}, 1, 0},
    {0xBA, (uint8_t []){0x0B}, 1, 0},
    {0xBB, (uint8_t []){0x02}, 1, 0},
    {0xBC, (uint8_t []){0xDB}, 1, 0},
    {0xBD, (uint8_t []){0x04}, 1, 0},
    {0xBE, (uint8_t []){0x00}, 1, 0},
    {0xBF, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x10}, 1, 0},
    {0xC1, (uint8_t []){0x47}, 1, 0},
    {0xC2, (uint8_t []){0x56}, 1, 0},
    {0xC3, (uint8_t []){0x65}, 1, 0},
    {0xC4, (uint8_t []){0x74}, 1, 0},
    {0xC5, (uint8_t []){0x88}, 1, 0},
    {0xC6, (uint8_t []){0x99}, 1, 0},
    {0xC7, (uint8_t []){0x01}, 1, 0},
    {0xC8, (uint8_t []){0xBB}, 1, 0},
    {0xC9, (uint8_t []){0xAA}, 1, 0},
    {0xD0, (uint8_t []){0x10}, 1, 0},
    {0xD1, (uint8_t []){0x47}, 1, 0},
    {0xD2, (uint8_t []){0x56}, 1, 0},
    {0xD3, (uint8_t []){0x65}, 1, 0},
    {0xD4, (uint8_t []){0x74}, 1, 0},
    {0xD5, (uint8_t []){0x88}, 1, 0},
    {0xD6, (uint8_t []){0x99}, 1, 0},
    {0xD7, (uint8_t []){0x01}, 1, 0},
    {0xD8, (uint8_t []){0xBB}, 1, 0},
    {0xD9, (uint8_t []){0xAA}, 1, 0},
    {0xF3, (uint8_t []){0x01}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0x21, (uint8_t []){}, 0, 0},
    {0x11, (uint8_t []){}, 0, 0},
    {0x00, (uint8_t []){}, 0, 120},
};
float tsens_value;
gpio_num_t AUDIO_I2S_GPIO_DIN = AUDIO_I2S_GPIO_DIN_1;
gpio_num_t AUDIO_CODEC_PA_PIN = AUDIO_CODEC_PA_PIN_1;
gpio_num_t QSPI_PIN_NUM_LCD_RST = QSPI_PIN_NUM_LCD_RST_1;
gpio_num_t TOUCH_PAD2 = TOUCH_PAD2_1;
gpio_num_t UART1_TX = UART1_TX_1;
gpio_num_t UART1_RX = UART1_RX_1;

class Charge : public I2cDevice {
public:
    Charge(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        read_buffer_ = new uint8_t[8];
    }
    ~Charge()
    {
        delete[] read_buffer_;
    }
    void Printcharge()
    {
        ReadRegs(0x08, read_buffer_, 2);
        ReadRegs(0x0c, read_buffer_ + 2, 2);
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));

        int16_t voltage = static_cast<uint16_t>(read_buffer_[1] << 8 | read_buffer_[0]);
        int16_t current = static_cast<int16_t>(read_buffer_[3] << 8 | read_buffer_[2]);
        
        // Use the variables to avoid warnings (can be removed if actual implementation uses them)
        (void)voltage;
        (void)current;
    }
    static void TaskFunction(void *pvParameters)
    {
        Charge* charge = static_cast<Charge*>(pvParameters);
        while (true) {
            charge->Printcharge();
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }

private:
    uint8_t* read_buffer_ = nullptr;
};

class Cst816s : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };

    enum TouchEvent {
        TOUCH_NONE,
        TOUCH_PRESS,
        TOUCH_RELEASE,
        TOUCH_HOLD
    };

    Cst816s(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        read_buffer_ = new uint8_t[6];
        was_touched_ = false;
        press_count_ = 0;

        // Create touch interrupt semaphore
        touch_isr_mux_ = xSemaphoreCreateBinary();
        if (touch_isr_mux_ == NULL) {
            ESP_LOGE("EchoEar", "Failed to create touch semaphore");
        }
    }

    ~Cst816s()
    {
        delete[] read_buffer_;

        // Delete semaphore if it exists
        if (touch_isr_mux_ != NULL) {
            vSemaphoreDelete(touch_isr_mux_);
            touch_isr_mux_ = NULL;
        }
    }

    void UpdateTouchPoint()
    {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    const TouchPoint_t &GetTouchPoint()
    {
        return tp_;
    }

    TouchEvent CheckTouchEvent()
    {
        bool is_touched = (tp_.num > 0);
        TouchEvent event = TOUCH_NONE;

        if (is_touched && !was_touched_) {
            // Press event (transition from not touched to touched)
            press_count_++;
            event = TOUCH_PRESS;
            ESP_LOGI("EchoEar", "TOUCH PRESS - count: %d, x: %d, y: %d", press_count_, tp_.x, tp_.y);
        } else if (!is_touched && was_touched_) {
            // Release event (transition from touched to not touched)
            event = TOUCH_RELEASE;
            ESP_LOGI("EchoEar", "TOUCH RELEASE - total presses: %d", press_count_);
        } else if (is_touched && was_touched_) {
            // Continuous touch (hold)
            event = TOUCH_HOLD;
            ESP_LOGD("EchoEar", "TOUCH HOLD - x: %d, y: %d", tp_.x, tp_.y);
        }

        // Update previous state
        was_touched_ = is_touched;
        return event;
    }

    int GetPressCount() const
    {
        return press_count_;
    }

    void ResetPressCount()
    {
        press_count_ = 0;
    }

    // Semaphore management methods
    SemaphoreHandle_t GetTouchSemaphore()
    {
        return touch_isr_mux_;
    }

    bool WaitForTouchEvent(TickType_t timeout = portMAX_DELAY)
    {
        if (touch_isr_mux_ != NULL) {
            return xSemaphoreTake(touch_isr_mux_, timeout) == pdTRUE;
        }
        return false;
    }

    void NotifyTouchEvent()
    {
        if (touch_isr_mux_ != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(touch_isr_mux_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;

    // Touch state tracking
    bool was_touched_;
    int press_count_;

    // Touch interrupt semaphore
    SemaphoreHandle_t touch_isr_mux_;
};

static bool gBaseCWPositive = true;

class RotatingBaseController {
public:
    RotatingBaseController()
    {
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT
        };
        uart_driver_install(UART_NUM_1, 1024, 1024, 0, NULL, 0);
        uart_param_config(UART_NUM_1, &uart_config);
        uart_set_pin(UART_NUM_1, UART1_TX, UART1_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        ESP_LOGI(TAG, "Base UART init TX=%d RX=%d baud=%d", (int)UART1_TX, (int)UART1_RX, uart_config.baud_rate);
    }
    ~RotatingBaseController()
    {
        uart_driver_delete(UART_NUM_1);
    }

    static RotatingBaseController& GetInstance() {
        static RotatingBaseController instance;
        return instance;
    }

    void RotateToAzimuth(int angle)
    {
        int a = angle;
        if (a < -180) a = -180;
        if (a > 180) a = 180;
        
        int64_t now_ms_gate = esp_timer_get_time() / 1000;
        if (s_last_sent_ >= 0) {
            int diff = a - s_last_raw_;
            if (diff < 0) diff = -diff;
            if (diff < 10 && (now_ms_gate - s_last_ms_) < 1500) return;
        }
        
        if (s_curr_mapped_ < 0) s_curr_mapped_ = 162;

        // Logic: Turn BY angle (Relative).
        // DOA angle is relative to current facing.
        // Target = Current + Angle.
        int mapped360 = s_curr_mapped_ + (gBaseCWPositive ? a : -a);
        if (mapped360 < 30) mapped360 = 30;
        if (mapped360 > 270) mapped360 = 270;
        
        SendFrame(mapped360);
        
        s_last_sent_ = mapped360;
        s_last_raw_ = a;
        s_last_ms_ = now_ms_gate;
        s_curr_mapped_ = mapped360; 
        
        ESP_LOGI(TAG, "Base UART rotate=%d mapped360=%d", a, mapped360);
    }

    void RotateByDelta(int delta)
    {
        int64_t now_ms_gate = esp_timer_get_time() / 1000;
        if (s_curr_mapped_ < 0) s_curr_mapped_ = 162;
        
        int dm = delta;
        if (dm < -180) dm = -180;
        if (dm > 180) dm = 180;
        
        if ((now_ms_gate - s_last_ms_) < 1500 && (dm > -10 && dm < 10)) return;
        
        int mapped360 = s_curr_mapped_ + (gBaseCWPositive ? dm : -dm);
        if (mapped360 < 30) mapped360 = 30;
        if (mapped360 > 270) mapped360 = 270;

        SendFrame(mapped360);
        
        s_curr_mapped_ = mapped360;
        s_last_ms_ = now_ms_gate;
        
        ESP_LOGI(TAG, "Base UART rotate_delta=%d mapped360=%d", delta, mapped360);
    }

    void RotateToAbsoluteAzimuth(int angle)
    {
        int a = angle;
        if (a < -180) a = -180;
        if (a > 180) a = 180;
        int64_t now_ms_gate = esp_timer_get_time() / 1000;
        int mapped360 = 162 + (gBaseCWPositive ? a : -a);
        if (mapped360 < 30) mapped360 = 30;
        if (mapped360 > 270) mapped360 = 270;
        SendFrame(mapped360);
        s_last_sent_ = mapped360;
        s_last_raw_ = a;
        s_last_ms_ = now_ms_gate;
        s_curr_mapped_ = mapped360;
        ESP_LOGI(TAG, "Base UART rotate_abs=%d mapped360=%d", a, mapped360);
    }

    int GetCurrentMapped() const { return s_curr_mapped_; }
    void SetDirectionPositive(bool v) { gBaseCWPositive = v; }

private:
    void SendFrame(int mapped360) {
        if (mapped360 < 30) mapped360 = 30;
        if (mapped360 > 270) mapped360 = 270;
        
        uart_set_baudrate(UART_NUM_1, 115200);
        uint8_t frame[8];
        frame[0] = 0xAA;
        frame[1] = 0x55;
        frame[2] = 0x00;
        frame[3] = 0x03;
        frame[4] = 0x01;
        uint16_t v = (uint16_t)mapped360;
        frame[5] = (uint8_t)((v >> 8) & 0xFF);
        frame[6] = (uint8_t)(v & 0xFF);
        frame[7] = (uint8_t)(frame[4] + frame[5] + frame[6]);
        uart_write_bytes(UART_NUM_1, (const char*)frame, 8);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    int s_last_sent_ = -1;
    int s_last_raw_ = 0;
    int64_t s_last_ms_ = 0;
    int s_curr_mapped_ = -1;
};

class SoundSourceLocator {
public:
    SoundSourceLocator(AudioCodec* codec) : codec_(codec)
    {
        std::string input_format("MM");
        doa_ = afe_doa_create(input_format.c_str(), 16000, 10.0f, 0.06f, 512);
        ESP_LOGI(TAG, "DOA init: sr=%d ch=%d", codec_->input_sample_rate(), codec_->input_channels());
    }
    ~SoundSourceLocator()
    {
        if (doa_) afe_doa_destroy(doa_);
    }
    static void TaskEntry(void* arg)
    {
        SoundSourceLocator* self = static_cast<SoundSourceLocator*>(arg);
        static int kDoaAngleOffset = 0;
        static bool kDoaInvert = false;
        static bool kLRInvert = false;
        static int kDoaMinPeak = 25;
        static int kDoaMinAngleChange = 6;
        static int kDoaMinIntervalMs = 300;
        static int kDoaMinPeakDiff = 15;
        static int kAngleSoftLimit = 120;
        static int kEntryMinPeak = 20;
        static int kEntryMinAngleChange = 8;
        static int kEntryWindowMs = 1500;
        static int kEntryMaxDelta = 60;
        static int kListeningMaxDelta = 45;
        static bool s_inited = false;
        static float s_avg = 0;
        static int s_last3[3] = {0,0,0};
        static int s_l3_count = 0;
        static int s_voice_frames = 0;
        static const int kVoiceGateFrames = 3;
        static int64_t s_last_voice_ms = 0;
        static bool s_rotated_session = false;
        static DeviceState s_prev_state = kDeviceStateUnknown;
        static int64_t s_listening_since_ms = 0;
        static bool s_prev_vad = false;
        static int64_t s_speech_start_ms = 0;
        static int64_t s_entry_until_ms = 0;
        static DeviceState s_entry_from_state = kDeviceStateUnknown;
        static int s_last_valid_angle = 0;
        static int64_t s_last_valid_ms = 0;
        static bool s_rotated_wake = false;
        static bool s_rotated_listening_round = false;
        static bool s_inhibit_until_speech = false;
        static bool s_rotated_speaking_round = false;
        static const int kEntryCacheMaxAgeMs = 300;
        static int s_bias_last3[3] = {0,0,0};
        static int s_bias_l3_count = 0;
        static bool s_orient_calibrated = false;
        static int s_orient_invert = 0;
        const size_t doa_window = 512; // samples per channel
        std::vector<int16_t> frame;
        frame.reserve(doa_window * 2);
        self->doa_buf_.clear();
        RotatingBaseController& base = RotatingBaseController::GetInstance();
            int64_t last_rotate_ms = 0;
            int prev_angle = 10000;
            int64_t ready_ms = esp_timer_get_time() / 1000 + 0;
            static int64_t s_cooldown_until_ms = 0;
            static int s_last_sign = 0;
            int64_t last_log_ms = 0;
            int empty_reads = 0;
            while (true) {
                frame.clear();
                DeviceState cur_state = Application::GetInstance().GetDeviceState();
            if (cur_state != s_prev_state) {
                bool is_new_session = false;
                bool is_continuation = false;
                if (cur_state == kDeviceStateListening) {
                    if (s_prev_state == kDeviceStateSpeaking) {
                        is_continuation = true;
                    } else {
                        is_new_session = true;
                    }
                } else if (cur_state == kDeviceStateSpeaking && s_prev_state == kDeviceStateIdle) {
                    is_new_session = true;
                }

        if (is_new_session) {
            s_rotated_session = false;
            s_rotated_wake = false;
            s_inhibit_until_speech = false;
            s_rotated_listening_round = false;
            s_inhibit_until_speech = false;
            s_rotated_speaking_round = false;
            s_voice_frames = 0;
            s_listening_since_ms = esp_timer_get_time() / 1000;
            s_entry_until_ms = s_listening_since_ms + kEntryWindowMs;
            s_entry_from_state = s_prev_state;
            self->doa_buf_.clear();
            s_inited = false;
            s_l3_count = 0;
            prev_angle = 10000;
            s_prev_vad = false;
            s_speech_start_ms = 0;
            s_orient_calibrated = false;
            s_orient_invert = 0;
            ESP_LOGI(TAG, "DOA Session Start: from %d to %d", s_prev_state, cur_state);
        } else if (is_continuation) {
            s_rotated_listening_round = false;
            s_rotated_wake = false;
            s_inhibit_until_speech = false;
            s_prev_vad = false;
            s_voice_frames = 0;
            s_listening_since_ms = esp_timer_get_time() / 1000;
            s_orient_calibrated = false;
            s_orient_invert = 0;
            ESP_LOGI(TAG, "DOA Session Continued: from %d to %d", s_prev_state, cur_state);
        } else {
            self->doa_buf_.clear();
        }
                if (cur_state == kDeviceStateSpeaking) {
                    s_rotated_speaking_round = false;
                }
                s_prev_state = cur_state;
            }
            if (!self->Read16kFrameFromCodec(frame, 160)) {
                empty_reads++;
                int64_t now_ms = esp_timer_get_time() / 1000;
                if (empty_reads % 50 == 0) {
                    ESP_LOGW(TAG, "DOA feed empty, count=%d", empty_reads);
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            int16_t peakL = 0, peakR = 0;
            for (size_t j = 0; j + 1 < frame.size(); j += 2) {
                int16_t l = frame[j];
                int16_t r = frame[j + 1];
                int16_t al = l >= 0 ? l : (int16_t)(-l);
                int16_t ar = r >= 0 ? r : (int16_t)(-r);
                if (al > peakL) peakL = al;
                if (ar > peakR) peakR = ar;
            }
            int64_t now_ms = esp_timer_get_time() / 1000;
            if (now_ms - last_log_ms >= 2000) {
                ESP_LOGI(TAG, "DOA feed L=%d R=%d buf=%d", peakL, peakR, (int)self->doa_buf_.size());
                last_log_ms = now_ms;
            }
            self->doa_buf_.insert(self->doa_buf_.end(), frame.begin(), frame.end());
            while (self->doa_buf_.size() >= doa_window * 2) {
                float az = afe_doa_process(self->doa_, self->doa_buf_.data());
                
                // Debug: Check for mono input (L=R)
                static int64_t last_mono_check = 0;
                int64_t now_check = esp_timer_get_time() / 1000;
                if (now_check - last_mono_check > 5000) {
                    int same_count = 0;
                    for (size_t k = 0; k < doa_window * 2; k += 2) {
                        if (self->doa_buf_[k] == self->doa_buf_[k+1]) same_count++;
                    }
                    if (same_count > doa_window / 2) {
                        ESP_LOGW(TAG, "DOA WARNING: High similarity between L/R channels! same_count=%d/%d", same_count, (int)doa_window);
                    }
                    last_mono_check = now_check;
                }

                int angle = (int)az;
                if (angle > 180) angle -= 360;

                int lr_bias_raw = (int)peakL - (int)peakR;
                int lr_bias = kLRInvert ? -lr_bias_raw : lr_bias_raw;
                int absang = angle >= 0 ? angle : -angle;
                int raw_sign = angle >= 0 ? 1 : -1;
                int bias_sign = (lr_bias > 0) ? -1 : ((lr_bias < 0) ? 1 : 0);
                int bias_mag = lr_bias >= 0 ? lr_bias : -lr_bias;
                int max_peak = (peakL > peakR ? peakL : peakR);
                int lr_bias_thresh = max_peak / 6;
                if (lr_bias_thresh < kDoaMinPeakDiff) lr_bias_thresh = kDoaMinPeakDiff;

                int bias_major = bias_sign;
                if (s_bias_l3_count < 3) {
                    s_bias_last3[s_bias_l3_count++] = bias_sign;
                } else {
                    s_bias_last3[0] = s_bias_last3[1];
                    s_bias_last3[1] = s_bias_last3[2];
                    s_bias_last3[2] = bias_sign;
                    int b0 = s_bias_last3[0], b1 = s_bias_last3[1], b2 = s_bias_last3[2];
                    int pos = (b0==1) + (b1==1) + (b2==1);
                    int neg = (b0==-1) + (b1==-1) + (b2==-1);
                    if (pos >= 2) bias_major = 1; else if (neg >= 2) bias_major = -1; else bias_major = 0;
                }

                int corrected = kDoaInvert ? -angle : angle;
                corrected += kDoaAngleOffset;
                if (corrected < -180) corrected = -180;
                if (corrected > 180) corrected = 180;
                
                int64_t now_ms = esp_timer_get_time() / 1000;
                if (peakL > kEntryMinPeak || peakR > kEntryMinPeak) {
                    s_last_valid_angle = corrected;
                    s_last_valid_ms = now_ms;
                    g_doa_last_angle = corrected;
                    g_doa_last_ms = now_ms;
            if (now_ms % 2000 < 50) {
                 ESP_LOGI(TAG, "DOA valid angle cached: %d (raw_az=%.1f) (peakL=%d peakR=%d)", corrected, az, peakL, peakR);
            }
            
            // wake-entry rotation: require robust LR bias to avoid wrong initial turn; only once per wake
            if ((now_ms <= s_entry_until_ms) && !s_rotated_wake) {
                int cmd_entry = corrected;
                if (cmd_entry < -kAngleSoftLimit) cmd_entry = -kAngleSoftLimit;
                if (cmd_entry >  kAngleSoftLimit) cmd_entry =  kAngleSoftLimit;
                if (cmd_entry > kEntryMaxDelta) cmd_entry = kEntryMaxDelta;
                if (cmd_entry < -kEntryMaxDelta) cmd_entry = -kEntryMaxDelta;
                ESP_LOGI(TAG, "DOA (wake-entry) azimuth=%.1f rotate_to=%d", az, cmd_entry);
                int before = base.GetCurrentMapped();
                base.RotateToAbsoluteAzimuth(cmd_entry);
                int after = base.GetCurrentMapped();
                int move = after - before; if (move < 0) move = -move;
                int move_sign = (after - before) > 0 ? 1 : ((after - before) < 0 ? -1 : 0);
                int cmd_sign = (cmd_entry > 0) ? 1 : ((cmd_entry < 0) ? -1 : 0);
                if (move_sign != 0 && cmd_sign != 0 && move_sign != cmd_sign) { base.SetDirectionPositive(!gBaseCWPositive); base.RotateToAbsoluteAzimuth(cmd_entry); }
                last_rotate_ms = now_ms;
                prev_angle = cmd_entry;
                s_cooldown_until_ms = now_ms + 4000;
                s_last_sign = 0;
                s_rotated_wake = true;
            }
        }

                if (now_ms < ready_ms) {
                    self->doa_buf_.erase(self->doa_buf_.begin(), self->doa_buf_.begin() + doa_window * 2);
                    continue;
                }
        DeviceState ds = Application::GetInstance().GetDeviceState();
        bool should_process = (ds == kDeviceStateIdle || ds == kDeviceStateListening || ds == kDeviceStateConnecting);
        if (ds == kDeviceStateSpeaking) {
            should_process = false;
            if (!s_rotated_wake && (now_ms <= s_entry_until_ms)) {
                should_process = true;
            }
        }
        
        // moved inline checks of entry window; no separate flag needed

        if (!should_process) {
            self->doa_buf_.clear();
            continue;
        }
        // Entry rotation handled above; no additional path here
                bool vad = Application::GetInstance().IsVoiceDetected();
                int lr_diff = (peakL > peakR ? (peakL - peakR) : (peakR - peakL));
                bool amp_gate = (peakL >= kDoaMinPeak || peakR >= kDoaMinPeak);
                bool voice_gate = (vad && amp_gate && (lr_diff >= kDoaMinPeakDiff));
                if (!voice_gate) {
                    s_voice_frames = 0;
                    self->doa_buf_.erase(self->doa_buf_.begin(), self->doa_buf_.begin() + doa_window * 2);
                    continue;
                } else {
                    if (s_voice_frames < 1000) s_voice_frames++;
                    s_last_voice_ms = esp_timer_get_time() / 1000;
                    if (!s_prev_vad && vad) {
                        s_speech_start_ms = s_last_voice_ms;
                    }
                }
                if (s_voice_frames < kVoiceGateFrames) {
                    self->doa_buf_.erase(self->doa_buf_.begin(), self->doa_buf_.begin() + doa_window * 2);
                    continue;
                }
                int cmdAngle = corrected;
                if (cmdAngle < -kAngleSoftLimit) cmdAngle = -kAngleSoftLimit;
                if (cmdAngle >  kAngleSoftLimit) cmdAngle =  kAngleSoftLimit;
                
                
        
                // removed secondary entry delta rotation to ensure single entry move
                int sdiff = (prev_angle > 9000) ? 0 : (cmdAngle - prev_angle); if (sdiff < 0) sdiff = -sdiff;
                // Previously blocked rotation here if s_rotated_session was true. Removed to allow continuous tracking.
                
                if (vad && !s_prev_vad) {
                    if (prev_angle > 9000) {
                        prev_angle = cmdAngle;
                        s_prev_vad = vad;
                        self->doa_buf_.erase(self->doa_buf_.begin(), self->doa_buf_.begin() + doa_window * 2);
                        continue;
                    }
                }
                if (((ds == kDeviceStateListening) || (ds == kDeviceStateSpeaking && (now_ms - s_speech_start_ms) <= 300))
                    && vad && !s_prev_vad && !s_inhibit_until_speech && !s_rotated_listening_round &&
                    Application::GetInstance().GetAudioService().IsAudioProcessorRunning() &&
                    (now_ms - s_listening_since_ms) >= 150 && (now_ms - last_rotate_ms) >= kDoaMinIntervalMs && now_ms >= s_cooldown_until_ms) {
                    if (cmdAngle < -kAngleSoftLimit) cmdAngle = -kAngleSoftLimit;
                    if (cmdAngle >  kAngleSoftLimit) cmdAngle =  kAngleSoftLimit;
                    if (cmdAngle > kListeningMaxDelta) cmdAngle = kListeningMaxDelta;
                    if (cmdAngle < -kListeningMaxDelta) cmdAngle = -kListeningMaxDelta;
                    ESP_LOGI(TAG, "DOA decision (listening): raw_sign=%d bias_major=%d corrected=%d cmd=%d prev=%d", raw_sign, bias_major, corrected, cmdAngle, prev_angle);
                    int before_m = base.GetCurrentMapped();
                    base.RotateToAbsoluteAzimuth(cmdAngle);
                    int after_m = base.GetCurrentMapped();
                    int move_sign_m = (after_m - before_m) > 0 ? 1 : ((after_m - before_m) < 0 ? -1 : 0);
                    int cmd_sign_m = (cmdAngle > 0) ? 1 : ((cmdAngle < 0) ? -1 : 0);
                    if (move_sign_m != 0 && cmd_sign_m != 0 && move_sign_m != cmd_sign_m) { base.SetDirectionPositive(!gBaseCWPositive); base.RotateToAbsoluteAzimuth(cmdAngle); }
                    last_rotate_ms = now_ms;
                    prev_angle = cmdAngle;
                    s_cooldown_until_ms = now_ms + 1500;
                    s_rotated_listening_round = true;
                    s_prev_vad = vad;
                    s_inhibit_until_speech = true;
                }
                // speaking 阶段不做 DOA 跟随，避免影响识别
                s_prev_vad = vad;
                self->doa_buf_.erase(self->doa_buf_.begin(), self->doa_buf_.begin() + doa_window * 2);
            }
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    }
private:
    bool Read16kFrameFromCodec(std::vector<int16_t>& out, size_t samples)
    {
        int in_sr = codec_->input_sample_rate();
        int channels = codec_->input_channels();
        size_t in_samples = samples * in_sr / 16000 * channels;
        temp_.resize(in_samples);
        if (!codec_->InputData(temp_)) {
            return false;
        }
        out.resize(samples * 2);
        static bool kSwapChannels = false;
        if (channels >= 2 && in_sr == 16000) {
            for (size_t i = 0, j = 0; i < samples; ++i, j += 2) {
                if (!kSwapChannels) {
                    out[i*2] = temp_[j];
                    out[i*2+1] = temp_[j+1];
                } else {
                    out[i*2] = temp_[j+1];
                    out[i*2+1] = temp_[j];
                }
            }
            return true;
        }
        for (size_t i = 0; i < samples; ++i) {
            float pos = (float)i * (float)in_sr / 16000.0f;
            size_t idx = (size_t)pos;
            if (idx + 1 >= (temp_.size() / channels)) idx = (temp_.size() / channels) - 2;
            float frac = pos - (float)idx;
            int16_t s0L = temp_[idx*channels + 0];
            int16_t s1L = temp_[(idx+1)*channels + 0];
            int16_t s0R = (channels >= 2) ? temp_[idx*channels + 1] : s0L;
            int16_t s1R = (channels >= 2) ? temp_[(idx+1)*channels + 1] : s1L;
            int32_t vL = (int32_t)((1.0f - frac) * s0L + frac * s1L);
            int32_t vR = (int32_t)((1.0f - frac) * s0R + frac * s1R);
            out[i*2] = (int16_t)vL;
            out[i*2+1] = (int16_t)vR;
        }
        return true;
    }
    AudioCodec* codec_ = nullptr;
    afe_doa_handle_t* doa_ = nullptr;
    std::vector<int16_t> doa_buf_;
    std::vector<int16_t> temp_;
};

class Mpu6050Imu : public I2cDevice {
public:
    Mpu6050Imu(i2c_master_bus_handle_t bus, uint8_t addr) : I2cDevice(bus, addr) {}
    bool Initialize()
    {
        uint8_t who = ReadReg(0x75);
        if (who == 0x68 || who == 0x70) {
            WriteReg(0x6B, 0x00);
            return true;
        }
        return false;
    }
    bool ReadAccel(float& ax, float& ay, float& az)
    {
        uint8_t buf[6];
        ReadRegs(0x3B, buf, 6);
        int16_t x = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t y = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t z = (int16_t)((buf[4] << 8) | buf[5]);
        float scale = 16384.0f;
        ax = (float)x / scale;
        ay = (float)y / scale;
        az = (float)z / scale;
        return true;
    }
};

    class Bma423Imu : public I2cDevice {
public:
    Bma423Imu(i2c_master_bus_handle_t bus, uint8_t addr) : I2cDevice(bus, addr), bus_(bus), addr_(addr) {}
    bool Initialize()
    {
        // Reattach(100000, 0); // Removed unconditional reattach to prevent bus reset loop
        uint8_t chip = 0;
        uint8_t r0 = 0x00;
        for (int i = 0; i < 3; i++) {
            if (i2c_master_transmit_receive(i2c_device_, &r0, 1, &chip, 1, 100) == ESP_OK) {
                if (chip != 0x00 && chip != 0xFF) break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        ESP_LOGI("BMA423", "Probing addr 0x%02X, Chip ID: 0x%02X", addr_, chip);
        if (chip != 0x13 && chip != 0x1F) { // Enforce BMA423 Chip ID (allow 0x1F as potential variant)
            ESP_LOGE("BMA423", "Invalid Chip ID (expected 0x13), attempting reattach...");
            Reattach(100000, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            if (i2c_master_transmit_receive(i2c_device_, &r0, 1, &chip, 1, 100) != ESP_OK || (chip != 0x13 && chip != 0x1F)) {
                ESP_LOGE("BMA423", "Still Invalid Chip ID after reattach: 0x%02X", chip);
                return false;
            }
            ESP_LOGI("BMA423", "Chip ID recovered: 0x%02X", chip);
        }
        
        // Soft reset
        uint8_t w2[2] = {0x7E, 0xB6}; 
        if (i2c_master_transmit(i2c_device_, w2, 2, 100) != ESP_OK) {
             ESP_LOGE("BMA423", "Soft reset tx failed");
             return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Increased wait for soft reset
        
        uint8_t w3[2] = {0x7C, 0x00}; // Power save disable
        if (i2c_master_transmit(i2c_device_, w3, 2, 100) != ESP_OK) {
            ESP_LOGE("BMA423", "Power save disable failed");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
        
        uint8_t w4[2] = {0x40, 0xA8}; // Accel config: ODR 100Hz, Normal, Perf mode
        uint8_t w5[2] = {0x41, 0x00}; // Accel range +/- 2g
        if (i2c_master_transmit(i2c_device_, w4, 2, 100) != ESP_OK) {
             ESP_LOGE("BMA423", "Accel config failed");
             return false;
        }
        if (i2c_master_transmit(i2c_device_, w5, 2, 100) != ESP_OK) {
             ESP_LOGE("BMA423", "Accel range failed");
             return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        
        uint8_t pwr = 0;
        uint8_t rp = 0x7D;
        if (i2c_master_transmit_receive(i2c_device_, &rp, 1, &pwr, 1, 100) == ESP_OK) {
            uint8_t w6[2] = {0x7D, (uint8_t)(pwr | 0x04)}; // Enable Accel
            if (i2c_master_transmit(i2c_device_, w6, 2, 100) != ESP_OK) {
                 ESP_LOGE("BMA423", "Enable accel tx failed");
                 return false;
            }
        } else {
            ESP_LOGE("BMA423", "Failed to read PWR_CTRL");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Verify enabled
        uint8_t check_pwr = 0;
        i2c_master_transmit_receive(i2c_device_, &rp, 1, &check_pwr, 1, 100);
        if ((check_pwr & 0x04) == 0) {
             ESP_LOGW("BMA423", "Accel enable failed? PWR=0x%02X", check_pwr);
             // Try one more time
             uint8_t w6[2] = {0x7D, 0x04};
             i2c_master_transmit(i2c_device_, w6, 2, 100);
             vTaskDelay(pdMS_TO_TICKS(10));
             i2c_master_transmit_receive(i2c_device_, &rp, 1, &check_pwr, 1, 100);
             if ((check_pwr & 0x04) == 0) {
                 ESP_LOGE("BMA423", "Accel enable retry failed PWR=0x%02X", check_pwr);
                 return false;
             }
        }

        inited_ = true;
        return true;
    }
    bool ReadAccel(float& ax, float& ay, float& az)
    {
        uint8_t buf[6];
        uint8_t reg = 0x12;
        if (i2c_master_transmit_receive(i2c_device_, &reg, 1, buf, 6, 100) != ESP_OK) {
            return false;
        }
        
        // Check for invalid data (all 00 or all FF)
        bool all_zeros = true;
        bool all_ff = true;
        for(int i=0; i<6; i++) {
            if (buf[i] != 0) all_zeros = false;
            if (buf[i] != 0xFF) all_ff = false;
        }
        if (all_zeros || all_ff) return false;

        int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

        // BMA423 is 12-bit, left aligned in 16-bit registers if shadowing is enabled?
        // Actually usually it's signed 16-bit where the lower 4 bits are 0.
        // Wait, BMA423 datasheet:
        // Register 0x12: ACC_X_LSB [7:4] = acc_x[3:0]
        // Register 0x13: ACC_X_MSB [7:0] = acc_x[11:4]
        // Value = (MSB * 256 + LSB) / 16
        
        x >>= 4;
        y >>= 4;
        z >>= 4;

        // Range +/- 2g (assuming 0x41=0x00 in Init) -> 1 LSB = 0.98mg
        // 2g range: 1024 LSB/g
        ax = (float)x / 1024.0f;
        ay = (float)y / 1024.0f;
        az = (float)z / 1024.0f;
        
        if (fabsf(ax) > 3.9f || fabsf(ay) > 3.9f || fabsf(az) > 3.9f) return false;
        return true;
    }

    bool EnableAccel(uint8_t acc_conf, uint8_t acc_range)
    {
        if (!inited_) return false;
        // Force 100Hz ODR to ensure data ready
        acc_conf = 0xA8; // 100Hz, Normal mode
        
        uint8_t w1[2] = {0x40, acc_conf};
        uint8_t w2[2] = {0x41, acc_range};
        i2c_master_transmit(i2c_device_, w1, 2, 100);
        i2c_master_transmit(i2c_device_, w2, 2, 100);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        uint8_t pwr = 0;
        uint8_t rp = 0x7D;
        // Read current power mode
        if (i2c_master_transmit_receive(i2c_device_, &rp, 1, &pwr, 1, 100) == ESP_OK) {
            // Enable Accel (bit 2)
            uint8_t w3[2] = {0x7D, (uint8_t)(pwr | 0x04)};
            i2c_master_transmit(i2c_device_, w3, 2, 100);
        } else {
            // Blind write if read fails
            uint8_t w3[2] = {0x7D, 0x04};
            i2c_master_transmit(i2c_device_, w3, 2, 100);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        return true;
    }
    bool ReadRaw6(uint8_t* out)
    {
        uint8_t r = 0x12;
        if (i2c_master_transmit_receive(i2c_device_, &r, 1, out, 6, 1000) != ESP_OK) return false;
        return true;
    }
    bool Reattach(uint32_t scl_hz, uint16_t scl_wait_us)
    {
        if (i2c_device_) {
            (void)i2c_master_bus_rm_device(i2c_device_);
            i2c_device_ = nullptr;
        }
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr_,
            .scl_speed_hz = scl_hz,
            .scl_wait_us = scl_wait_us,
            .flags = {.disable_ack_check = 0},
        };
        esp_err_t err = i2c_master_bus_add_device(bus_, &cfg, &i2c_device_);
        vTaskDelay(pdMS_TO_TICKS(2));
        return err == ESP_OK;
    }
private:
    bool inited_ = false;
    i2c_master_bus_handle_t bus_;
    uint8_t addr_;
};

class EspS3Cat : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Cst816s* cst816s_;
    Charge* charge_;
    Button boot_button_;
    Display* display_ = nullptr;
    PwmBacklight* backlight_ = nullptr;
    esp_timer_handle_t touchpad_timer_;
    esp_lcd_touch_handle_t tp;   // LCD touch handle
    esp_timer_handle_t doa_start_timer_ = nullptr;
    
    bool doa_enabled_ = true;
    bool imu_diag_isolation_ = false;

    void InitializeI2c()
    {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
        ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    }
    void RecoverI2cBus()
    {
        i2c_del_master_bus(i2c_bus_);
        gpio_config_t cfg;
        cfg.pin_bit_mask = (1ULL << AUDIO_CODEC_I2C_SCL_PIN) | (1ULL << AUDIO_CODEC_I2C_SDA_PIN);
        cfg.mode = GPIO_MODE_OUTPUT_OD;
        cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&cfg);
        gpio_set_level(AUDIO_CODEC_I2C_SCL_PIN, 1);
        gpio_set_level(AUDIO_CODEC_I2C_SDA_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(2));
        for (int i = 0; i < 16; ++i) {
            gpio_set_level(AUDIO_CODEC_I2C_SCL_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(1));
            gpio_set_level(AUDIO_CODEC_I2C_SCL_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        gpio_set_level(AUDIO_CODEC_I2C_SDA_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(AUDIO_CODEC_I2C_SCL_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(AUDIO_CODEC_I2C_SDA_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(2));
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    // Removed PowerResetImu(): it toggled POWER_CTRL (GPIO9, speaker PA rail),
    // which does not power the IMU (it stays alive through the pulse) but did
    // disturb the display/audio supply and caused visible flicker. IMU reset is
    // now done with the BMI2 soft reset inside BMI270Imu::Initialize().

    uint8_t DetectPcbVersion()
    {
        esp_err_t ret = i2c_master_probe(i2c_bus_, 0x18, 100);
        uint8_t pcb_verison = 0;
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "PCB verison V1.0");
            pcb_verison = 0;
        } else {
            gpio_config_t gpio_conf = {
                .pin_bit_mask = (1ULL << GPIO_NUM_48),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            ESP_ERROR_CHECK(gpio_config(&gpio_conf));
            ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_48, 1));
            vTaskDelay(pdMS_TO_TICKS(100));
            ret = i2c_master_probe(i2c_bus_, 0x18, 100);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "PCB verison V1.2");
                pcb_verison = 1;
                AUDIO_I2S_GPIO_DIN = AUDIO_I2S_GPIO_DIN_2;
                AUDIO_CODEC_PA_PIN = AUDIO_CODEC_PA_PIN_2;
                QSPI_PIN_NUM_LCD_RST = QSPI_PIN_NUM_LCD_RST_2;
                TOUCH_PAD2 = TOUCH_PAD2_2;
                UART1_TX = UART1_TX_2;
                UART1_RX = UART1_RX_2;
            } else {
                ESP_LOGE(TAG, "PCB version detection error");

            }
        }
        return pcb_verison;
    }

    static void touch_isr_callback(void* arg)
    {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (touchpad != nullptr) {
            touchpad->NotifyTouchEvent();
        }
    }

    static void touch_event_task(void* arg)
    {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (touchpad == nullptr) {
            ESP_LOGE(TAG, "Invalid touchpad pointer in touch_event_task");
            vTaskDelete(NULL);
            return;
        }

        while (true) {
            if (touchpad->WaitForTouchEvent()) {
                auto &app = Application::GetInstance();
                auto &board = (EspS3Cat &)Board::GetInstance();

                ESP_LOGI(TAG, "Touch event, TP_PIN_NUM_INT: %d", gpio_get_level(TP_PIN_NUM_INT));
                touchpad->UpdateTouchPoint();
                auto touch_event = touchpad->CheckTouchEvent();

                if (touch_event == Cst816s::TOUCH_RELEASE) {
                    if (app.GetDeviceState() == kDeviceStateStarting &&
                            !WifiStation::GetInstance().IsConnected()) {
                        board.ResetWifiConfiguration();
                    } else {
                        app.ToggleChatState();
                    }
                }
            }
        }
    }

    void InitializeCharge()
    {
        esp_err_t ret = i2c_master_probe(i2c_bus_, 0x55, 100);
        if (ret == ESP_OK) {
            charge_ = new Charge(i2c_bus_, 0x55);
            xTaskCreatePinnedToCore(Charge::TaskFunction, "batterydecTask", 3 * 1024, charge_, 6, NULL, 0);
        } else {
            ESP_LOGW(TAG, "Charge IC not found on I2C addr 0x55, skip charge task");
        }
    }

    void InitializeCst816sTouchPad()
    {
        cst816s_ = new Cst816s(i2c_bus_, 0x15);

        xTaskCreatePinnedToCore(touch_event_task, "touch_task", 4 * 1024, cst816s_, 5, NULL, 1);

        const gpio_config_t int_gpio_config = {
            .pin_bit_mask = (1ULL << TP_PIN_NUM_INT),
            .mode = GPIO_MODE_INPUT,
            // .intr_type = GPIO_INTR_NEGEDGE
            .intr_type = GPIO_INTR_ANYEDGE
        };
        gpio_config(&int_gpio_config);
        gpio_install_isr_service(0);
        gpio_intr_enable(TP_PIN_NUM_INT);
        gpio_isr_handler_add(TP_PIN_NUM_INT, EspS3Cat::touch_isr_callback, cst816s_);
    }

    void InitializeSpi()
    {
        const spi_bus_config_t bus_config = TAIJIPI_ST77916_PANEL_BUS_QSPI_CONFIG(QSPI_PIN_NUM_LCD_PCLK,
                                                                                  QSPI_PIN_NUM_LCD_DATA0,
                                                                                  QSPI_PIN_NUM_LCD_DATA1,
                                                                                  QSPI_PIN_NUM_LCD_DATA2,
                                                                                  QSPI_PIN_NUM_LCD_DATA3,
                                                                                  QSPI_LCD_H_RES * 80 * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    void Initializest77916Display(uint8_t pcb_verison)
    {

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        const esp_lcd_panel_io_spi_config_t io_config = ST77916_PANEL_IO_QSPI_CONFIG(QSPI_PIN_NUM_LCD_CS, NULL, NULL);
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io));
        st77916_vendor_config_t vendor_config = {
            .init_cmds = vendor_specific_init_yysj,
            .init_cmds_size = sizeof(vendor_specific_init_yysj) / sizeof(st77916_lcd_init_cmd_t),
            .flags = {
                .use_qspi_interface = 1,
            },
        };
        const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = QSPI_PIN_NUM_LCD_RST,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = QSPI_LCD_BIT_PER_PIXEL,
            .flags = {
                .reset_active_high = pcb_verison,
            },
            .vendor_config = &vendor_config,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_disp_on_off(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
#else
        display_ = new SpiLcdDisplay(panel_io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
        backlight_ = new PwmBacklight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        backlight_->RestoreBrightness();
    }

    void InitializeButtons()
    {
        boot_button_.OnClick([this]() {
            auto &app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ESP_LOGI(TAG, "Boot button pressed, enter WiFi configuration mode");
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
        
        gpio_config_t power_gpio_config = {
            .pin_bit_mask = (BIT64(POWER_CTRL)),
            .mode = GPIO_MODE_OUTPUT,

        };
        ESP_ERROR_CHECK(gpio_config(&power_gpio_config));

        gpio_set_level(POWER_CTRL, 0);
    }

    static void imu_task(void* arg)
    {
        // Delay IMU task to avoid I2C conflict during system startup (AudioCodec init, etc.)
        vTaskDelay(pdMS_TO_TICKS(5000));

        EspS3Cat* self = static_cast<EspS3Cat*>(arg);
        // Bma423Imu bma18(self->i2c_bus_, 0x18); // 0x18 is ES8311 on this board
        Bma423Imu bma19(self->i2c_bus_, 0x19);
        // Bma423Imu bma15(self->i2c_bus_, 0x15); // Occupied by CST816S Touch
        Bma423Imu* bma = nullptr;
        Mpu6050Imu mpu(self->i2c_bus_, 0x68);
        BMI270Imu bmi270(self->i2c_bus_, 0x68);
        bool use_bma = false;
        uint8_t bma_addr = 0x00;
        bool ok = false;
        bool bmi270_present = false;
        bool bma_present = false;
        int bmi_zero_trials = 0;
        int64_t s_boot_ms = esp_timer_get_time()/1000;
        static int s_fail_log_cnt = 0;
        bool s_bmi_raw_valid = false;
        // gpio_set_level(POWER_CTRL, 1);
        // vTaskDelay(pdMS_TO_TICKS(20));
        // gpio_set_level(POWER_CTRL, 0);
        // vTaskDelay(pdMS_TO_TICKS(20));
        // I2C scan removed to reduce bus contention during boot
        if (i2c_master_probe(self->i2c_bus_, 0x68, 100) == ESP_OK) {
            bmi270.SetAddress(0x68);
            bool init_ok = bmi270.Initialize();
            uint8_t chip = 0; bmi270.ReadRaw(0x00, &chip, 1);
            const char* cname = (chip == 0x24 ? "BMI270" : (chip == 0x26 ? "BMI26x" : "BMI2"));
            ESP_LOGI(TAG, "IMU device: %s addr=0x68 id=0x%02X", cname, chip);
            use_bma = false; ok = init_ok; bmi270_present = init_ok;
            if (init_ok) s_bmi_raw_valid = true;
            if (!init_ok) {
                if (mpu.Initialize()) {
                    ESP_LOGW(TAG, "BMI init failed, fallback to MPU6050 (0x68)");
                    ok = true; bmi270_present = false; bma_present = false;
                } else if (i2c_master_probe(self->i2c_bus_, 0x19, 100) == ESP_OK && bma19.Initialize()) {
                    ESP_LOGW(TAG, "BMI init failed, fallback to BMA423 (0x19)");
                    bma = &bma19; bma->EnableAccel(0xA8, 0x00);
                    bma_present = true; ok = true; bmi270_present = false;
                }
            }
        } else if (i2c_master_probe(self->i2c_bus_, 0x69, 100) == ESP_OK) {
            bmi270.SetAddress(0x69);
            bool init_ok = bmi270.Initialize();
            uint8_t chip2 = 0; bmi270.ReadRaw(0x00, &chip2, 1);
            const char* cname2 = (chip2 == 0x24 ? "BMI270" : (chip2 == 0x26 ? "BMI26x" : "BMI2"));
            ESP_LOGI(TAG, "IMU device: %s addr=0x69 id=0x%02X", cname2, chip2);
            use_bma = false; ok = init_ok; bmi270_present = init_ok;
            if (init_ok) s_bmi_raw_valid = true;
            if (!init_ok) {
                if (mpu.Initialize()) {
                    ESP_LOGW(TAG, "BMI init failed, fallback to MPU6050 (0x69)");
                    ok = true; bmi270_present = false; bma_present = false;
                } else if (i2c_master_probe(self->i2c_bus_, 0x19, 100) == ESP_OK) {
                    if (bma19.Initialize()) {
                        ESP_LOGW(TAG, "BMI init failed, fallback to BMA423 (0x19)");
                        bma = &bma19; bma_addr = 0x19;
                        bma->EnableAccel(0xA8, 0x00);
                        bma_present = true; ok = true; bmi270_present = false;
                    }
                }
            }
        } else if (i2c_master_probe(self->i2c_bus_, 0x19, 100) == ESP_OK) {
            if (bma19.Initialize()) {
                ESP_LOGI(TAG, "IMU device: BMA423 addr=0x19");
                bma = &bma19; bma_addr = 0x19;
                bma_present = true; ok = true; bmi270_present = false;
            }
        }
        auto display = Board::GetInstance().GetDisplay();
        int last_emote_time = 0;
        int last_emote_code = 0;
        float prev_ax = 0, prev_ay = 0, prev_az = 0;
        float avg_ax = 0.0f, avg_ay = 0.0f, avg_az = 0.0f;
        bool s_filter_init = false;
        float prev_mag = 0;
        int shake_score = 0;
        
        int64_t last_dizzy_ms = 0;
        int64_t s_silly_until_ms = 0;   // deadline for the one-shot "silly" animation
        int64_t last_imu_log_ms = 0;
        bool imu_selected_logged = false;
        int64_t imu_start_ms = esp_timer_get_time()/1000;
        int64_t last_retry_log_ms = 0;
        static int64_t s_last_bmi_init_ms = 0;
        static int s_bmi_init_attempts = 0;
        static bool s_mpu_inited = false;
        static int64_t s_i2c_quiet_until_ms = 0;
        static int64_t s_next_bmi_try_ms = 0;
        static int s_bmi_backoff_stage = 0;
        static int64_t s_imu_status_log_ms = 0;
        static int64_t s_coherence_quiet_until_ms = 0;
        // Last time any IMU recovery action was performed. Used to arbitrate
        // between the many recovery paths so they cannot reset the device in
        // rapid succession (which previously caused flicker and lost shaking).
        static int64_t s_last_recovery_ms = 0;
        static int s_recovery_count = 0;

            // Single arbitration point for every IMU recovery action.
            // Returns false when a recovery happened recently, so callers must
            // simply retry later instead of resetting again.
            // NOTE: never toggle POWER_CTRL (GPIO9) here. It powers only the
            // speaker PA on this board and the IMU stays powered; toggling it
            // produced screen/audio rail glitches while doing nothing for the IMU.
            auto try_imu_recover = [&](bool hard, const char* reason) -> bool {
                int64_t now_ms = esp_timer_get_time()/1000;
                if (s_i2c_quiet_until_ms > now_ms) return false;
                if (s_last_recovery_ms != 0 && (now_ms - s_last_recovery_ms) < 5000) return false;
                if (s_recovery_count >= 10) {
                    // Too many recoveries in a row: back off hard to stop a loop.
                    s_i2c_quiet_until_ms = now_ms + 30000;
                    bmi270.SetDriverQuiet(now_ms + 30000);
                    s_recovery_count = 0;
                    ESP_LOGW(TAG, "IMU recovery budget exhausted, quiet 30s (%s)", reason);
                    return false;
                }
                ESP_LOGW(TAG, "IMU recovery: %s (hard=%d count=%d)", reason, (int)hard, s_recovery_count);
                if (hard) {
                    // Low-speed reattach plus the inline soft reset in Initialize().
                    bmi270.Reattach(25000, 20);
                }
                bool ok_reinit = bmi270.Initialize();
                if (ok_reinit) {
                    bmi270.EnsureEnabled();
                    s_bmi_raw_valid = true;
                } else {
                    s_bmi_raw_valid = false;
                }
                s_last_recovery_ms = now_ms;
                s_recovery_count++;
                s_bmi_init_attempts++;
                s_last_bmi_init_ms = now_ms;
                // Short quiet window: enough to let the bus settle, short enough
                // that shaking during recovery is not lost.
                s_i2c_quiet_until_ms = now_ms + 2000;
                bmi270.SetDriverQuiet(now_ms + 2000);
                if (!ok_reinit) {
                    bmi270_present = false;
                    s_next_bmi_try_ms = now_ms + 10000;
                }
                return true;
            };
        int stuck_count = 0;
        while (true) {
            if (!ok) {
                int64_t now_retry = esp_timer_get_time()/1000;
                if (now_retry - last_retry_log_ms > 30000) { ESP_LOGD(TAG, "IMU not initialized, retrying..."); last_retry_log_ms = now_retry; }
                vTaskDelay(pdMS_TO_TICKS(500));
                int64_t now_gate = esp_timer_get_time()/1000;
                if (s_i2c_quiet_until_ms > now_gate) {
                    ok = false;
                } else {
                    bool tried = false;
                    if (i2c_master_probe(self->i2c_bus_, 0x68, 100) == ESP_OK) {
                        bmi270.SetAddress(0x68);
                        tried = true;
                        if (bmi270.Initialize()) {
                            ESP_LOGI(TAG, "IMU device: BMI270/BMI26x addr=0x68 (forced)");
                            use_bma = false; ok = true; bmi270_present = true;
                        } else {
                            ESP_LOGD(TAG, "EchoEar: BMI270 Init failed at addr 0x68");
                            ok = false;
                        }
                    }
                    if (!ok && i2c_master_probe(self->i2c_bus_, 0x69, 100) == ESP_OK) {
                        bmi270.SetAddress(0x69);
                        tried = true;
                        if (bmi270.Initialize()) {
                            ESP_LOGI(TAG, "IMU device: BMI270/BMI26x addr=0x69 (forced)");
                            use_bma = false; ok = true; bmi270_present = true;
                        } else {
                            ESP_LOGD(TAG, "EchoEar: BMI270 Init failed at addr 0x69");
                            ok = false;
                        }
                    }
                    if (!tried) ok = false;
                }
                if (!ok && bmi270_present) { if (now_retry - last_retry_log_ms > 30000) { ESP_LOGD(TAG, "IMU BMI270 present but not enabled, retrying"); last_retry_log_ms = now_retry; } }
                if (!ok) {
                    int64_t now_stat_ms = esp_timer_get_time()/1000;
                    if (now_stat_ms - s_imu_status_log_ms > 1000) {
                        int quiet_ms = (s_i2c_quiet_until_ms > now_stat_ms) ? (int)(s_i2c_quiet_until_ms - now_stat_ms) : 0;
                        ESP_LOGI(TAG, "IMU status: present(bmi=%d bma=%d) read_ok=0 quiet_remain=%dms zero_trials=%d raw_valid=%d",
                            bmi270_present, bma_present, quiet_ms, bmi_zero_trials, (int)s_bmi_raw_valid);
                        s_imu_status_log_ms = now_stat_ms;
                        static int64_t s_next_diag_ms = 0;
                if (quiet_ms == 0 && (s_next_diag_ms == 0 || now_stat_ms >= s_next_diag_ms)) {
                    uint8_t who = 0;
                    bmi270.Reattach(50000, 12);
                    if (bmi270.ReadRaw(0x00, &who, 1)) {
                        ESP_LOGI(TAG, "IMU diag WHOAMI: 0x%02X", who);
                        if (who == 0x26) {
                            uint8_t rp=0, er=0, stx=0;
                            (void)bmi270.ReadRaw(0x7D, &rp, 1);
                            (void)bmi270.ReadRaw(0x02, &er, 1);
                            (void)bmi270.ReadRaw(0x1B, &stx, 1);
                            ESP_LOGI(TAG, "IMU diag REGS PWR=0x%02X ERR=0x%02X STAT=0x%02X", rp, er, stx);
                            bool recovered = bmi270.TryRecoverData();
                            if (!recovered) {
                                bmi270.Initialize();
                                bmi270.EnsureEnabled();
                            }
                                    int drdy = 0;
                                    int drdy_int0 = 0;
                                    for (int k = 0; k < 10; ++k) {
                                        uint8_t st = 0;
                                        if (who == 0x26) {
                                            if (bmi270.ReadRaw(0x1C, &st, 1) && (st & 0xC0)) { drdy_int0 = 1; break; }
                                        } else {
                                            if (bmi270.ReadRaw(0x1B, &st, 1) && (st & 0xC0)) { drdy = 1; break; }
                                        }
                                        vTaskDelay(pdMS_TO_TICKS(50));
                                    }
                                    if (who == 0x26) ESP_LOGI(TAG, "IMU diag DRDY_INT0=%d", drdy_int0);
                                    else ESP_LOGI(TAG, "IMU diag DRDY=%d", drdy);
                                    if (!drdy) {
                                        int lodr_drdy = 0;
                                        bmi270.EnableAccel(0xA4, 0x00);
                                        vTaskDelay(pdMS_TO_TICKS(80));
                                        for (int k = 0; k < 10; ++k) {
                                            uint8_t st3 = 0;
                                            if (bmi270.ReadRaw(0x1B, &st3, 1) && (st3 & 0xC0)) { lodr_drdy = 1; break; }
                                            vTaskDelay(pdMS_TO_TICKS(50));
                                        }
                                        ESP_LOGI(TAG, "IMU diag LODR_DRDY=%d", lodr_drdy);
                                        if (lodr_drdy) {
                                            bmi270.EnableAccel(0xA8, 0x00);
                                            vTaskDelay(pdMS_TO_TICKS(80));
                                            bmi270_present = true;
                                            ok = true;
                                        }
                                        if (!lodr_drdy) {
                                            int rec_drdy = 0;
                                            bmi270.Reattach(25000, 20);
                                            bmi270.Initialize();
                                            bmi270.EnsureEnabled();
                                            for (int k = 0; k < 20; ++k) {
                                                uint8_t st4 = 0;
                                                if (bmi270.ReadRaw(0x1B, &st4, 1) && (st4 & 0xC0)) { rec_drdy = 1; break; }
                                                vTaskDelay(pdMS_TO_TICKS(60));
                                            }
                                            ESP_LOGI(TAG, "IMU diag RECOVERY_DRDY=%d", rec_drdy);
                                            if (rec_drdy) { bmi270_present = true; ok = true; } else {
                                                // No POWER_CTRL toggle: it does not power the IMU
                                                // (IMU stayed alive through reset pulses) and it
                                                // caused the screen/audio rail glitches.
                                                int pd_drdy = 0;
                                                bmi270.Reattach(25000, 12);
                                                bmi270.Initialize();
                                                bmi270.EnsureEnabled();
                                                for (int k = 0; k < 30; ++k) {
                                                    uint8_t st5 = 0;
                                                    if (bmi270.ReadRaw(0x1B, &st5, 1) && (st5 & 0xC0)) { pd_drdy = 1; break; }
                                                    vTaskDelay(pdMS_TO_TICKS(80));
                                                }
                                                ESP_LOGI(TAG, "IMU diag PWRRESET_DRDY=%d", pd_drdy);
                                                if (pd_drdy) { bmi270_present = true; ok = true; }
                                            }
                                        }
                                    }
                                    if (drdy) {
                                        bmi270_present = true;
                                        ok = true;
                                    }
                                }
                            }
                            s_next_diag_ms = now_stat_ms + 30000;
                        }
                    }
                }
                continue;
            }
                static int64_t imu_select_ms = 0;
                if (!imu_selected_logged) {
                if (bmi270_present) {
                    uint8_t chip3 = 0; bmi270.ReadRaw(0x00, &chip3, 1);
                    const char* cname3 = (chip3 == 0x24 ? "BMI270" : (chip3 == 0x26 ? "BMI26x" : "BMI2"));
                    ESP_LOGI(TAG, "IMU device: %s addr=0x68 id=0x%02X", cname3, chip3);
                } else {
                    if (bma_present) {
                        ESP_LOGI(TAG, "IMU device: BMA423 addr=0x18");
                    } else {
                        ESP_LOGI(TAG, "IMU device: MPU6050 addr=0x68");
                    }
                }
                imu_selected_logged = true;
                imu_select_ms = esp_timer_get_time()/1000;
                }
            // Initialize explicitly: a frame without a successful read must never
            // hand garbage around. When the 3 s read-throttle skipped a sample,
            // these locals were left uninitialized and the downstream shake
            // detectors consumed the stack contents as if they were real
            // acceleration, which made "shake" fire while the device sat still.
            float ax = 0.0f, ay = 0.0f, az = 0.0f;
            bool read_ok = false;
            // True only when this iteration obtained a real sample from a sensor.
            bool sample_ok = false;
            static int bmi_fail_count = 0;
            static int64_t s_imu_data_log_ms = 0;
            
            // Always enter block to allow fallback logic
            if (true) {
                // A read skipped because of a recovery/backoff window is NOT a
                // failure: counting it here re-armed the circuit breaker and kept
                // the quiet window alive forever (shaking could never accumulate).
                bool read_was_skipped = false;
                if (bmi270_present) {
                    int64_t now_ms_gate = esp_timer_get_time()/1000;
                    if (s_i2c_quiet_until_ms > now_ms_gate) {
                        read_ok = false;
                        read_was_skipped = true;
                    } else {
                        // Sample every iteration (~80 ms). The old 3 s throttle here
                        // belongs to the diagnostic log below, not to the sampling
                        // path: it made shake detection respond only once per 3 s.
                        read_ok = bmi270.ReadAccel(ax, ay, az);
                        if (read_ok && !(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
                            s_bmi_raw_valid = true;
                        }
                    }
                    if (read_ok) sample_ok = true;
                }
                int64_t now_stat_ms = esp_timer_get_time()/1000;
                if (now_stat_ms - s_imu_status_log_ms > 1000) {
                    int quiet_ms = (s_i2c_quiet_until_ms > now_stat_ms) ? (int)(s_i2c_quiet_until_ms - now_stat_ms) : 0;
                    ESP_LOGI(TAG, "IMU status: present(bmi=%d bma=%d) read_ok=%d quiet_remain=%dms zero_trials=%d raw_valid=%d", 
                        bmi270_present, bma_present, (int)read_ok, quiet_ms, bmi_zero_trials, (int)s_bmi_raw_valid);
                    s_imu_status_log_ms = now_stat_ms;
                }
                if (!read_ok) {
                    // Only a real I2C timeout counts; a skipped read (quiet window
                    // or backoff) must not advance the failure/staged-recovery state.
                    if (!read_was_skipped && bmi270_present) {
                        bmi_fail_count++;
                    }
                    if (!read_was_skipped && bmi_fail_count >= 8) {
                        int64_t now_ms_cb = esp_timer_get_time()/1000;
                        s_i2c_quiet_until_ms = now_ms_cb + 60000;
                        ESP_LOGW(TAG, "IMU circuit-breaker engaged: quiet 60s due to repeated I2C timeouts");
                        bmi_fail_count = 0;
                    } else if (bmi_fail_count > 2) {
                        int64_t now_ms = esp_timer_get_time()/1000;
                        if ((now_ms - s_last_bmi_init_ms) > 5000) {
                            if (s_bmi_init_attempts < 2) {
                                if (bmi270.Initialize()) {
                                    s_bmi_raw_valid = true;
                                    ESP_LOGI(TAG, "BMI270 re-initialized successfully");
                                } else {
                                    ESP_LOGW(TAG, "BMI270 re-initialization failed");
                                    s_i2c_quiet_until_ms = now_ms + 5000; // backoff 5s to reduce I2C errors
                                }
                                s_last_bmi_init_ms = now_ms;
                                s_bmi_init_attempts++;
                            } else {
                                ESP_LOGW(TAG, "BMI270 persistent failure, scheduling hard reset/backoff and switching to BMA");
                                bool recovered = try_imu_recover(true, "persistent failure");
                                int64_t now_ms2 = esp_timer_get_time()/1000;
                                if (!bma_present) {
                                    if (i2c_master_probe(self->i2c_bus_, 0x19, 100) == ESP_OK && bma19.Initialize()) {
                                        bma = &bma19; bma_addr = 0x19; bma_present = true; bma->EnableAccel(0xA8, 0x00);
                                    }
                                }
                                if (!recovered) {
                                    // Arbiter refused (just recovered): keep BMI and retry later.
                                    s_next_bmi_try_ms = now_ms2 + 5000;
                                } else if (bmi270_present) {
                                    // Re-init succeeded: keep BMI as the primary source.
                                    ESP_LOGI(TAG, "BMI270 recovered by arbiter, staying on BMI");
                                } else {
                                    int backoffs[] = {10000, 30000, 60000, 120000};
                                    s_next_bmi_try_ms = now_ms2 + backoffs[s_bmi_backoff_stage];
                                    if (s_bmi_backoff_stage < 3) s_bmi_backoff_stage++;
                                    s_bmi_raw_valid = false;
                                }
                            }
                        }
                        bmi_fail_count = 0;
                    }
                } else {
                    bmi_fail_count = 0;
                }
                if (!read_ok && bma_present && bma) {
                    float bax, bay, baz;
                    if (bma->ReadAccel(bax, bay, baz)) {
                        ax = bax; ay = bay; az = baz;
                        read_ok = true;
                        int64_t nqw = esp_timer_get_time()/1000;
                        s_coherence_quiet_until_ms = nqw + 1500;
                    }
                    if (read_ok) sample_ok = true;
                } else if (!read_ok && !bma_present) {
                    if (i2c_master_probe(self->i2c_bus_, 0x19, 100) == ESP_OK && bma19.Initialize()) {
                        bma = &bma19; bma_addr = 0x19; bma_present = true; bma->EnableAccel(0xA8, 0x00);
                        float bax, bay, baz;
                        if (bma->ReadAccel(bax, bay, baz)) {
                            ax = bax; ay = bay; az = baz;
                            read_ok = true;
                            int64_t nqw2 = esp_timer_get_time()/1000;
                            s_coherence_quiet_until_ms = nqw2 + 1500;
                        }
                    }
                }
                if (!read_ok) {
                    float mx, my, mz;
                    if (mpu.ReadAccel(mx, my, mz)) {
                        ax = mx; ay = my; az = mz;
                        read_ok = true;
                        int64_t nqw3 = esp_timer_get_time()/1000;
                        s_coherence_quiet_until_ms = nqw3 + 1500;
                    }
                }
                if (!read_ok || (ax == 0.0f && ay == 0.0f && az == 0.0f)) {
                    bmi_zero_trials++;
                    if (bmi_zero_trials >= 2) {
                        bmi270.EnsureEnabled();
                    }
                    // No real sample this round: downstream detectors must not run.
                    sample_ok = false;
                } else {
                    bmi_zero_trials = 0;
                    sample_ok = true;   // a real sample reached this point
                }
                static const bool kAllowBmaSupplement = true;
                static int64_t s_next_bma_supp_ms = 0;
                    if (kAllowBmaSupplement && bmi_zero_trials >= 3) {
                        int64_t now_ms_bma = esp_timer_get_time()/1000;
                        if (s_next_bma_supp_ms != 0 && now_ms_bma < s_next_bma_supp_ms) {
                            // rate limit
                        } else {
                            s_next_bma_supp_ms = now_ms_bma + 10000;
                        if (!bma_present) {
                            if (i2c_master_probe(self->i2c_bus_, 0x19, 100) == ESP_OK && bma19.Initialize()) {
                                bma = &bma19; bma_addr = 0x19;
                                bma_present = true;
                                bma->EnableAccel(0xA8, 0x00);
                            }
                        }
                        if (bma_present) {
                            bool got = false;
                            float bax = ax, bay = ay, baz = az;
                            if (bma && bma->ReadAccel(bax, bay, baz)) {
                                got = true;
                            }
                            if (!got) {
                                if (bma) {
                                    if (bma->ReadAccel(bax, bay, baz)) {
                                        got = true;
                                    }
                                }
                            }
                        if (got) {
                            ax = bax; ay = bay; az = baz;
                            read_ok = true;
                            sample_ok = true;
                            int64_t now = esp_timer_get_time()/1000;
                            s_coherence_quiet_until_ms = now + 1500;
                            if (now - last_imu_log_ms > 1000) {
                                if (bma) { uint8_t rb[6]; bma->ReadRaw6(rb);
                                ESP_LOGI(TAG, "BMA ACC sup=%02X%02X%02X%02X%02X%02X ax=%.2f ay=%.2f az=%.2f",
                                    rb[0], rb[1], rb[2], rb[3], rb[4], rb[5], ax, ay, az); }
                            }
                        }
                    }
                    }
                }
                if (read_ok) {
                    float mag_est = fabsf(ax) + fabsf(ay) + fabsf(az);
                    if (kAllowBmaSupplement && mag_est < 0.10f) {
                        if (!bma_present && i2c_master_probe(self->i2c_bus_, 0x19, 100) == ESP_OK) {
                            if (bma19.Initialize()) {
                                bma = &bma19;
                                bma_present = true;
                                bma->EnableAccel(0xA8, 0x00);
                            }
                        }
                        if (bma_present && bma) {
                            float bax, bay, baz;
                            if (bma->ReadAccel(bax, bay, baz)) {
                                ax = bax; ay = bay; az = baz;
                            }
                        }
                    }
                    int64_t now_log_ms = esp_timer_get_time()/1000;
                    if (now_log_ms - s_imu_data_log_ms > 2000) {
                        ESP_LOGI(TAG, "IMU ax=%.2f ay=%.2f az=%.2f mag=%.2f", ax, ay, az, mag_est);
                        s_imu_data_log_ms = now_log_ms;
                    }
                }
                if (bmi_zero_trials >= 6 || bmi_fail_count > 3) {
                    // Route through the arbiter instead of re-initializing blindly:
                    // two independent re-init paths fighting each other was one of
                    // the causes of the reset storm.
                    (void)try_imu_recover(false, "zero/fail threshold");
                    bmi_zero_trials = 0;
                    bmi_fail_count = 0;
                }
            }
            if (!bmi270_present) {
                if (bma_present && bma) {
                    read_ok = bma->ReadAccel(ax, ay, az);
                    if (read_ok) {
                        // Check for all-zeros which might indicate a stuck sensor or unpowered
                        static int s_bma_zero_cnt = 0;
                        if (fabsf(ax) < 0.001f && fabsf(ay) < 0.001f && fabsf(az) < 0.001f) {
                            s_bma_zero_cnt++;
                            if (s_bma_zero_cnt > 50) {
                                ESP_LOGW(TAG, "BMA423 all zeros for >1s, re-initializing...");
                                bma->Initialize();
                                bma->EnableAccel(0xA8, 0x00);
                                s_bma_zero_cnt = 0;
                            }
                            if ((s_bma_zero_cnt % 50) == 0) {
                                ESP_LOGW(TAG, "BMA423 data all zeros? cnt=%d", s_bma_zero_cnt);
                            }
                            read_ok = false; 
                            sample_ok = false;
                        } else {
                            s_bma_zero_cnt = 0;
                            sample_ok = true;
                        }
                    } else {
                         static int s_bma_fail_cnt = 0;
                         if ((++s_bma_fail_cnt % 50) == 0) {
                             ESP_LOGW(TAG, "BMA423 ReadAccel failed");
                         }
                    }
                } else {
                    if (!s_mpu_inited) {
                        s_mpu_inited = mpu.Initialize();
                        if (s_mpu_inited) {
                            ESP_LOGI(TAG, "IMU device: MPU6050 addr=0x68");
                        }
                    }
                    read_ok = mpu.ReadAccel(ax, ay, az);
                    if (read_ok) {
                        sample_ok = true;
                    }
                }
            }
            // Only a frame that carried a real sample may feed the filters and the
            // shake detectors; otherwise garbage/stale values would look like motion.
            if (!sample_ok) {
                read_ok = false;
            }
            if (read_ok) {
                s_fail_log_cnt = 0;
                int64_t now = esp_timer_get_time()/1000;
                
                float cur_mag = fabsf(ax) + fabsf(ay) + fabsf(az);

                // Reset filters on startup or when recovering from zeros to avoid false shake trigger
                if (!s_filter_init || (cur_mag > 0.5f && (fabsf(avg_ax) + fabsf(avg_ay) + fabsf(avg_az)) < 0.1f)) {
                    avg_ax = ax; avg_ay = ay; avg_az = az;
                    prev_ax = ax; prev_ay = ay; prev_az = az;
                    s_filter_init = true;
                }

                float jerk = fabsf(ax - prev_ax) + fabsf(ay - prev_ay) + fabsf(az - prev_az);
                float mag = fabsf(ax) + fabsf(ay) + fabsf(az);
                float mag_diff = fabsf(mag - prev_mag);
                
                bool fallback_mode = !s_bmi_raw_valid;
                // alpha=0.99 gave a ~100-frame (~8 s) time constant, so after any
                // level change the EMA lagged for seconds and "dyn" froze at a
                // constant offset (0.057...0.075) even with perfectly still data
                // (jerk==0.000). That standing offset was then treated as motion.
                float alpha = (fallback_mode ? 0.90f : 0.95f);
                float one_minus_alpha = 1.0f - alpha;
                float prev_avg_ax = avg_ax, prev_avg_ay = avg_ay, prev_avg_az = avg_az;
                avg_ax = alpha * avg_ax + one_minus_alpha * ax;
                avg_ay = alpha * avg_ay + one_minus_alpha * ay;
                avg_az = alpha * avg_az + one_minus_alpha * az;
                // Measure change of the mean (AC) rather than the absolute
                // deviation (DC): a static reading and a settled EMA both give 0.
                float dyn = fabsf(avg_ax - prev_avg_ax) +
                            fabsf(avg_ay - prev_avg_ay) +
                            fabsf(avg_az - prev_avg_az);
                if (jerk < 0.001f && mag_diff > 0.06f) {
                    float capped = (mag_diff > 0.08f) ? 0.08f : mag_diff;
                    jerk = capped;
                }
                
                // Update previous values for next iteration's jerk calculation
                prev_ax = ax; prev_ay = ay; prev_az = az;

                static int s_bma_stuck_count = 0;
                static int s_bma_conf_index = 0;
                static int s_bma_range_index = 0;
                static int s_bma_recover_stage = 0;
                static const uint8_t kBmaConfs[3] = {0xA8, 0xA4, 0xAC};
                static const uint8_t kBmaRanges[4] = {0x00, 0x01, 0x02, 0x03};
                if (bma_present) {
                    bool saturated = (fabsf(ax) > 3.5f && fabsf(ay) > 3.5f && fabsf(az) > 3.5f);
                    // bool no_dyn = (dyn < 0.002f && jerk < 0.004f);
                    if (saturated) {
                        s_bma_stuck_count++;
                    } else if (s_bma_stuck_count > 0) {
                        s_bma_stuck_count--;
                    }
                    if (s_bma_stuck_count >= 25) {
                        uint32_t hz = (s_bma_recover_stage == 0 ? 100000u : (s_bma_recover_stage == 1 ? 50000u : 25000u));
                        uint16_t w = (s_bma_recover_stage == 0 ? 5u : (s_bma_recover_stage == 1 ? 8u : 12u));
                        if (bma) { bma->Reattach(hz, w); }
                        if (bma) bma->Initialize();
                        uint8_t conf = kBmaConfs[s_bma_conf_index];
                        uint8_t rng = kBmaRanges[s_bma_range_index];
                        if (bma) bma->EnableAccel(conf, rng);
                        s_bma_conf_index = (s_bma_conf_index + 1) % 3;
                        s_bma_range_index = (s_bma_range_index + (s_bma_conf_index == 0 ? 1 : 0)) % 4;
                        ESP_LOGI(TAG, "BMA reconfig applied: conf=%02X range=%02X hz=%u wait=%u stage=%d", conf, rng, (unsigned)hz, (unsigned)w, s_bma_recover_stage);
                        s_bma_stuck_count = 0;
                        if (s_bma_recover_stage < 2) s_bma_recover_stage++; else s_bma_recover_stage = 0;
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    
                    // Check for frozen data (identical values)
                    static float s_last_ax = 0, s_last_ay = 0, s_last_az = 0;
                    static int s_frozen_cnt = 0;
                    if (ax == s_last_ax && ay == s_last_ay && az == s_last_az) {
                        s_frozen_cnt++;
                        if (s_frozen_cnt % 20 == 0) {
                             ESP_LOGW(TAG, "BMA423 potential frozen data cnt=%d ax=%.3f", s_frozen_cnt, ax);
                        }
                        if (s_frozen_cnt > 50) { // 1 second
                             ESP_LOGW(TAG, "BMA423 frozen data detected, forcing re-init...");
                             if (bma) {
                                 bma->Reattach(100000, 0);
                                 bma->Initialize();
                                 bma->EnableAccel(0xA8, 0x00);
                             }
                             s_frozen_cnt = 0;
                        }
                    } else {
                        s_frozen_cnt = 0;
                    }
                    s_last_ax = ax; s_last_ay = ay; s_last_az = az;
                }
                if (!bma_present && !bmi270_present) {
                    static float s_mpu_last_ax = 0, s_mpu_last_ay = 0, s_mpu_last_az = 0;
                    static int s_mpu_frozen_cnt = 0;
                    if (ax == s_mpu_last_ax && ay == s_mpu_last_ay && az == s_mpu_last_az) {
                        s_mpu_frozen_cnt++;
                        if (s_mpu_frozen_cnt > 50) {
                            mpu.Initialize();
                            s_filter_init = false;
                            s_mpu_frozen_cnt = 0;
                        }
                    } else {
                        s_mpu_frozen_cnt = 0;
                    }
                    s_mpu_last_ax = ax; s_mpu_last_ay = ay; s_mpu_last_az = az;
                }
                if (now - last_imu_log_ms > 1000) {
                    ESP_LOGI(TAG, "IMU jerk=%.3f dyn=%.3f ax=%.2f ay=%.2f az=%.2f score=%d", jerk, dyn, ax, ay, az, shake_score);
                    if (bmi270_present) {
                        static int64_t s_bmi_status_quiet_until_ms = 0;
                        int64_t nowms = esp_timer_get_time()/1000;
                        if (nowms >= s_bmi_status_quiet_until_ms) {
                            uint8_t acc[6]{};
                        uint8_t gyr[6]{};
                        uint8_t time0 = 0;
                        uint8_t buf[13]{};
                        bool ok_block = bmi270.ReadRaw(0x0C, buf, 13);
                        if (ok_block) {
                            memcpy(acc, &buf[0], 6); // 0x0C-0x11 is Accel
                            memcpy(gyr, &buf[6], 6); // 0x12-0x17 is Gyro
                            time0 = buf[12];
                        }
                        bool ok_acc = ok_block;
                        bool ok_gyr = ok_block;
                        bool ok_tm = ok_block;
                        if (ok_acc && ok_gyr) {
                            uint8_t pwr = 0, err = 0, st = 0, int_st = 0, chip_id = 0, acc_conf = 0;
                            bool ok_pwr = bmi270.ReadRaw(0x7D, &pwr, 1);
                            bool ok_err = bmi270.ReadRaw(0x02, &err, 1);
                            bool ok_st  = bmi270.ReadRaw(0x1B, &st, 1);
                            bool ok_int = bmi270.ReadRaw(0x21, &int_st, 1);
                            bool ok_id  = bmi270.ReadRaw(0x00, &chip_id, 1);
                            bool ok_cf  = bmi270.ReadRaw(0x40, &acc_conf, 1);
                            bool valid_data = ((acc[0]|acc[1]|acc[2]|acc[3]|acc[4]|acc[5]|gyr[0]|gyr[1]|gyr[2]|gyr[3]|gyr[4]|gyr[5]) != 0);
                            s_bmi_raw_valid = valid_data; // Only update validity on successful read
                            
                            if (valid_data) {
                                s_fail_log_cnt = 0;
                            }

                            ESP_LOGI(TAG, "BMI ACC=%02X%02X%02X%02X%02X%02X GYR=%02X%02X%02X%02X%02X%02X Pwr=%02X Time=%u ERR=%02X STAT=%02X INT_ST=%02X ID=%02X CONF=%02X VALID=%d",
                                acc[0], acc[1], acc[2], acc[3], acc[4], acc[5],
                                gyr[0], gyr[1], gyr[2], gyr[3], gyr[4], gyr[5],
                                ok_pwr ? pwr : 0, ok_tm ? time0 : 0, ok_err ? err : 0, ok_st ? st : 0,
                                ok_int ? int_st : 0, ok_id ? chip_id : 0, ok_cf ? acc_conf : 0, s_bmi_raw_valid);
                                static uint8_t s_bmi_last_time0 = 0;
                                static int s_bmi_time_stuck_sec = 0;
                                if (ok_tm) {
                                    if (time0 == s_bmi_last_time0) {
                                        s_bmi_time_stuck_sec++;
                                    } else {
                                        s_bmi_time_stuck_sec = 0;
                                    }
                                    s_bmi_last_time0 = time0;
                                }
                        if (s_bmi_time_stuck_sec >= 4) {
                            static int s_bmi_recover_attempts = 0;
                            // Escalating bus speed is handled by the arbiter's hard path;
                            // the old code also toggled POWER_CTRL here (speaker PA rail).
                            bool rec_ok = try_imu_recover(true, "sensor time stalled");
                            s_filter_init = false;
                            s_bmi_time_stuck_sec = 0;
                            if (rec_ok) s_bmi_recover_attempts++;
                            if (s_bmi_recover_attempts >= 3 && !s_bmi_raw_valid) {
                                ESP_LOGW(TAG, "BMI disabled due to persistent I2C failures; falling back to BMA/MPU");
                                bmi270_present = false;
                                // Force enable BMA as primary
                                if (!bma_present) {
                                    if (i2c_master_probe(self->i2c_bus_, 0x19, 100) == ESP_OK) {
                                        bma = &bma19; bma_addr = 0x19; bma_present = true;
                                    }
                                }
                                if (bma_present && bma) {
                                    ESP_LOGI(TAG, "Forcing BMA initialization for fallback...");
                                    bma->Initialize();
                                    bma->EnableAccel(0xAC, 0x00);
                                }
                            }
                        }
                            } else {
                                s_bmi_raw_valid = false;
                                s_bmi_status_quiet_until_ms = nowms + 5000;
                            }
                        }
                        static int64_t s_diag_tick_ms = 0;
                        int64_t nowdt = esp_timer_get_time()/1000;
                        if (s_diag_tick_ms == 0 || (nowdt - s_diag_tick_ms) >= 5000) {
                            uint8_t buf2[13]{};
                            bool ok2 = bmi270.ReadRaw(0x0C, buf2, 13);
                            int acc_nz = (buf2[0]|buf2[1]|buf2[2]|buf2[3]|buf2[4]|buf2[5]) != 0;
                            int gyr_nz = (buf2[6]|buf2[7]|buf2[8]|buf2[9]|buf2[10]|buf2[11]) != 0;
                            uint8_t st2 = 0;
                            (void)bmi270.ReadRaw(0x1B, &st2, 1);
                            ESP_LOGI(TAG, "IMU diag tick: ok=%d DRDY=%d ACC_NZ=%d GYR_NZ=%d", (int)ok2, (int)((st2 & 0xC0) ? 1 : 0), acc_nz, gyr_nz);
                            s_diag_tick_ms = nowdt;
                        }
                    }
                    if (bma_present) {
                        static i2c_master_dev_handle_t s_bma_mon = nullptr;
                        if (!s_bma_mon) {
                            i2c_device_config_t cfgm = {
                                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                .device_address = (uint16_t)(bma_addr ? bma_addr : 0x19),
                                .scl_speed_hz = 100 * 1000,
                                .scl_wait_us = 5,
                                .flags = {.disable_ack_check = 0},
                            };
                            i2c_master_bus_add_device(self->i2c_bus_, &cfgm, &s_bma_mon);
                        }
                        
                    }
                    last_imu_log_ms = now;
                }
                // ---- Shake window state -------------------------------------
                // s_high_energy is a LEAKY rectifier: high-frequency energy above
                //   the slow baseline. The old code used an unbounded accumulator
                //   (E * 0.7 + diff), and because "diff" is a SUM OF ABSOLUTES it
                //   has a DC offset, so E settled at ~3.3x diff (~0.27 at rest) and
                //   stayed there forever -> every round was judged as shaking.
                // s_baseline tracks the idle level (~10 s) so the threshold is
                //   relative, independent of range and of BMA fallback mode.
                static float s_high_energy = 0.0f;
                static float s_baseline = 0.0f;
                static int s_candidate_hits = 0;   // candidates inside the window
                static int s_flip_count = 0;
                static int s_last_sign = 0;
                static int64_t s_last_flip_ms = 0;
                static int64_t s_window_start_ms = 0;
                static int s_fast_trigger_cnt = 0;
                static int64_t s_fast_window_start_ms = 0;

                // Every counter is cleared together at the trigger point below;
                // previously several branches cleared only some of them, so state
                // leaked from one gesture into the next.

                bool shake_detected = false;
                bool in_quiet = (s_coherence_quiet_until_ms != 0 && now < s_coherence_quiet_until_ms);
                bool saturated_now = (fabsf(ax) > 3.5f && fabsf(ay) > 3.5f && fabsf(az) > 3.5f);
                // mag is |ax|+|ay|+|az|, so at rest it is ~1.0 (1 g). The old
                // "mag > 0.15" was true even when perfectly still and constrained
                // nothing; require a plausible gravity magnitude instead.
                bool mag_plausible = (fabsf(mag - 1.0f) < 0.60f) && (mag > 0.30f);

                float hf = fabsf(jerk) + fabsf(dyn);
                if (s_baseline <= 0.0f) {
                    // Seed on the first sample so a cold start does not look like
                    // an infinite spike (hf_excess would otherwise be 0 forever
                    // relative to a zero baseline).
                    s_baseline = (hf > 0.01f) ? hf : 0.01f;
                }
                s_baseline = s_baseline * 0.995f + hf * 0.005f;
                float hf_excess = (hf > s_baseline) ? (hf - s_baseline) : 0.0f;
                s_high_energy = s_high_energy * 0.85f + hf_excess * 0.15f;

                // dyn is now the frame-to-frame change of the mean (AC), so it is
                // roughly (1-alpha) times the instantaneous step: it is 0 when
                // still and smaller in magnitude than the old absolute deviation.
                float dyn_th = fallback_mode ? 0.015f : 0.020f;
                float jerk_th = fallback_mode ? 0.06f : 0.05f;
                bool candidate = ((dyn > dyn_th) || (jerk > jerk_th)) && !saturated_now &&
                                 mag_plausible && !in_quiet;
                bool emote_ready = false;
                if (auto emo = dynamic_cast<emote::EmoteDisplay*>(display)) {
                    auto d1 = emo->GetEmojiData(std::string("confused"));
                    auto d2 = emo->GetEmojiData(std::string("neutral"));
                    emote_ready = (d1.data != nullptr) && (d2.data != nullptr);
                }

                float axd = ax - avg_ax;
                float flip_th = fallback_mode ? 0.03f : 0.06f;
                int sign = (axd > flip_th) ? 1 : ((axd < -flip_th) ? -1 : 0);
                if (sign != 0) {
                    int64_t now_ms = esp_timer_get_time()/1000;
                    if (s_last_flip_ms == 0) s_last_flip_ms = now_ms;
                    if (s_last_sign != 0 && sign != s_last_sign && (now_ms - s_last_flip_ms) < 1200) {
                        s_flip_count++;
                    }
                    s_last_sign = sign;
                    s_last_flip_ms = now_ms;
                } else if (s_flip_count > 0) {
                    // Decay fast: a slow "-1 per frame" let idle noise keep the
                    // flip counter near the trigger level indefinitely.
                    s_flip_count = (s_flip_count > 2) ? (s_flip_count - 2) : 0;
                }

                // Sliding window (300 ms) instead of a +1/-1 running counter: the
                // old counter only needed 4 weak frames in a row to fire.
                int64_t now_win_ms = esp_timer_get_time()/1000;
                if (s_window_start_ms == 0 || (now_win_ms - s_window_start_ms) > 300) {
                    s_window_start_ms = now_win_ms;
                    s_candidate_hits = 0;
                }
                if (candidate) s_candidate_hits++;

                bool energy_spike = (s_high_energy > 3.0f * s_baseline) && (s_high_energy > 0.08f);
                bool motion_confirmed = (s_candidate_hits >= 3) || (s_flip_count >= 3);
                if (emote_ready && energy_spike && motion_confirmed && mag_plausible && !in_quiet) {
                    shake_detected = true;
                }
                float axis_delta = fabsf(ax - prev_ax) + fabsf(ay - prev_ay) + fabsf(az - prev_az);
                bool fast_candidate = fallback_mode && mag_plausible && (
                    ((jerk > 0.08f || dyn > 0.010f) && axis_delta > 0.08f) ||
                    (mag_diff > 0.10f && axis_delta > 0.08f)
                ) && !in_quiet;
                int64_t now_ms_fc = esp_timer_get_time()/1000;
                if (fast_candidate) {
                    if (s_fast_window_start_ms == 0 || (now_ms_fc - s_fast_window_start_ms) > 1000) {
                        s_fast_window_start_ms = now_ms_fc;
                        s_fast_trigger_cnt = 0;
                    }
                    s_fast_trigger_cnt++;
                } else {
                    if (s_fast_window_start_ms != 0 && (now_ms_fc - s_fast_window_start_ms) > 1000) {
                        s_fast_window_start_ms = 0;
                        s_fast_trigger_cnt = 0;
                    }
                }
                if (!shake_detected && s_fast_trigger_cnt >= 3 && fallback_mode && mag_plausible) {
                    shake_detected = true;
                }
                // Do not suppress shake purely due to zero vectors when fallback is active; rely on any-motion and fast window
                if (bmi270_present && ax == 0.0f && ay == 0.0f && az == 0.0f && !fallback_mode) {
                    shake_detected = false;
                }
                if (bmi270_present && !shake_detected && s_bmi_raw_valid) {
                    // Same sample as the task-level path: no extra I2C read.
                    if (emote_ready && mag_plausible && bmi270.UpdateShakeDetectionWith(ax, ay, az)) {
                        shake_detected = true;
                        ESP_LOGI(TAG, "IMU shake: energy path trigger");
                    }
                }
                if (bmi270_present && !shake_detected) {
                    static int64_t s_next_anymotion_check_ms = 0;
                    int64_t now_ms_any = esp_timer_get_time()/1000;
                    bool can_check_any = (s_next_anymotion_check_ms == 0 || now_ms_any >= s_next_anymotion_check_ms) && (s_i2c_quiet_until_ms <= now_ms_any);
                    // Single hardware check: CheckAnyMotionRaw() polled the same
                    // INT_STATUS_0 bit as CheckAnyMotion() with a second I2C read.
                    if (can_check_any && emote_ready && s_bmi_raw_valid && bmi270.CheckAnyMotion() &&
                        (axis_delta > 0.08f || dyn > 0.015f)) {
                        shake_detected = true;
                        ESP_LOGI(TAG, "IMU shake: any-motion trigger");
                        s_next_anymotion_check_ms = now_ms_any + 3000;
                    } else {
                        if (!can_check_any) {
                            // skip check due to quiet or gating
                        } else {
                            s_next_anymotion_check_ms = now_ms_any + 1500;
                        }
                    }
                }
                if ((now - imu_select_ms) < 2000) {
                    shake_detected = false;
                }
                if ((now - s_boot_ms) < 6000) {
                    shake_detected = false;
                }

                if (shake_detected && (now - last_dizzy_ms) > 1200) {
                    bool shown = false;
                    if (auto emo = dynamic_cast<emote::EmoteDisplay*>(display)) {
                        auto assets = emo->GetEmojiData(std::string("confused"));
                        if (assets.data) {
                            display->SetEmotion("confused");
                            shown = true;
                        } else {
                            auto nassets = emo->GetEmojiData(std::string("neutral"));
                            if (nassets.data) {
                                display->SetEmotion("neutral");
                                shown = true;
                            }
                        }
                    }
                    if (!shown) {
                        shown = true;
                    }
                    ESP_LOGI(TAG, "IMU silly triggered (shake)");
                    last_dizzy_ms = now;
                    last_emote_time = (int)now;
                    last_emote_code = 7;
                    // "silly" is a one-shot animation; without a deadline the eyes
                    // would freeze on its last frame until another emotion arrived.
                    s_silly_until_ms = now + 2500;
                    // Clear every counter at once so one gesture cannot fire again
                    // from leftover state.
                    s_high_energy = 0.0f;
                    s_candidate_hits = 0;
                    s_flip_count = 0;
                    s_last_sign = 0;
                    s_last_flip_ms = 0;
                    s_window_start_ms = 0;
                    s_fast_trigger_cnt = 0;
                    s_fast_window_start_ms = 0;
                }
                // Auto-restore neutral once the silly animation has run its course.
                if (s_silly_until_ms != 0 && now >= s_silly_until_ms) {
                    s_silly_until_ms = 0;
                    if (auto emo = dynamic_cast<emote::EmoteDisplay*>(display)) {
                        if (emo->GetEmojiData(std::string("neutral")).data) {
                            display->SetEmotion("neutral");
                            last_emote_code = 0;
                            ESP_LOGI(TAG, "IMU silly finished, restore neutral");
                        }
                    }
                }
                if (stuck_count >= 10) {
                    if (bmi270_present) {
                        bmi270.Initialize();
                        s_filter_init = false;
                    } else {
                        ok = mpu.Initialize();
                    }
                    stuck_count = 0;
                    imu_selected_logged = false;
                }
                static int s_no_dyn_cnt = 0;
                if (bma_present) {
                    if (dyn < 0.005f && jerk < 0.005f) {
                        s_no_dyn_cnt++;
                        if (s_no_dyn_cnt >= 50) {
                            if (bma) {
                                bma->Reattach(100000, 5);
                                bma->Initialize();
                                bma->EnableAccel(0xA8, 0x00);
                            }
                            s_filter_init = false;
                            s_no_dyn_cnt = 0;
                        }
                    } else {
                        s_no_dyn_cnt = 0;
                    }
                }
                if (!bmi270_present && bma_present && s_no_dyn_cnt >= 50) {
                    // s_no_dyn_cnt is the real "no dynamics" counter maintained
                    // above; the old s_static_count here was never incremented,
                    // so this recovery path could not fire.
                    if (bma) {
                        bma->Initialize();
                        bma->EnableAccel(0xA8, 0x00);
                    }
                    s_no_dyn_cnt = 0;
                }
                prev_ax = ax; prev_ay = ay; prev_az = az; prev_mag = mag;
            }

            if (!read_ok) {
                s_filter_init = false;
                if ((++s_fail_log_cnt % 200) == 0) {
                    ESP_LOGW(TAG, "IMU read failed (cnt=%d) bmi=%d bma=%d", s_fail_log_cnt, bmi270_present, bma_present);
                    // If BMI270 fails consistently, disable it to allow fallback to BMA
                    if (bmi270_present && s_fail_log_cnt >= 10) {
                        ESP_LOGE(TAG, "BMI270 unstable, scheduling hard reset and BMA fallback");
                        bool recovered = try_imu_recover(true, "unstable reads");
                        int64_t now_ms3 = esp_timer_get_time()/1000;
                        if (!recovered) {
                            s_next_bmi_try_ms = now_ms3 + 5000;
                        } else if (!bmi270_present) {
                            int backoffs[] = {10000, 30000, 60000, 120000};
                            s_next_bmi_try_ms = now_ms3 + backoffs[s_bmi_backoff_stage];
                            if (s_bmi_backoff_stage < 3) s_bmi_backoff_stage++;
                        }
                        if (!bma_present) {
                            vTaskDelay(pdMS_TO_TICKS(20));
                            if (i2c_master_probe(self->i2c_bus_, 0x19, 100) == ESP_OK && bma19.Initialize()) {
                                bma = &bma19; bma_addr = 0x19; bma_present = true; bma->EnableAccel(0xA8, 0x00);
                                ESP_LOGI(TAG, "BMA423 enabled at 0x19 (fallback)");
                            } else {
                                if (!s_mpu_inited && mpu.Initialize()) {
                                    s_mpu_inited = true;
                                    ESP_LOGW(TAG, "Fallback to MPU6050");
                                }
                            }
                        }
                        s_fail_log_cnt = 0;
                    }
                    // If BMA is active but failing (e.g. all zeros), try to re-init
                    if (!bmi270_present && bma_present && s_fail_log_cnt >= 20) {
                        ESP_LOGE(TAG, "BMA423 unstable/zeros, attempting re-init");
                        if (bma) {
                            bma->Reattach(100000, 0);
                            bma->Initialize();
                            bma->EnableAccel(0xA8, 0x00);
                        }
                        s_fail_log_cnt = 0;
                    }
                }
            }
            // Escalating backoff for BMI retries to reduce I2C timeouts
            int64_t now_loop_ms = esp_timer_get_time()/1000;
            if (!bmi270_present && !bma_present) {
                if (s_next_bmi_try_ms == 0 || now_loop_ms >= s_next_bmi_try_ms) {
                    if (s_i2c_quiet_until_ms > now_loop_ms) {
                        // still in quiet window, skip
                    } else {
                        // Last-resort re-acquisition on 0x68/0x69. The arbiter
                        // rate-limits this so the loop cannot reset every cycle.
                        int64_t rec_ms = esp_timer_get_time()/1000;
                        if (s_last_recovery_ms != 0 && (rec_ms - s_last_recovery_ms) < 5000) {
                            // too soon, wait
                        } else {
                        s_last_recovery_ms = rec_ms;
                        bmi270.SetAddress(0x68);
                        bool ok68 = bmi270.Initialize();
                        bool recovered = false;
                        if (ok68) {
                            recovered = true;
                        } else {
                            bmi270.SetAddress(0x69);
                            if (bmi270.Initialize()) recovered = true;
                        }
                        if (recovered) {
                            uint8_t who = 0; bmi270.ReadRaw(0x00, &who, 1);
                            ESP_LOGI(TAG, "BMI WHOAMI=0x%02X", who);
                            s_bmi_backoff_stage = 0;
                            s_bmi_init_attempts = 0;
                            s_bmi_raw_valid = true;
                            bmi270_present = true;
                            s_next_bmi_try_ms = 0;
                            ESP_LOGI(TAG, "BMI270/BMI26x recovered after backoff");
                        } else {
                            int backoffs2[] = {30000, 60000, 120000, 180000};
                            s_next_bmi_try_ms = now_loop_ms + backoffs2[s_bmi_backoff_stage];
                            if (s_bmi_backoff_stage < 3) s_bmi_backoff_stage++;
                            s_i2c_quiet_until_ms = now_loop_ms + 5000;
                        }
                        }
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS((bmi270_present || bma_present) ? 80 : 350));
        }
    }

public:
    EspS3Cat() : boot_button_(BOOT_BUTTON_GPIO)
    {
        InitializeI2c();
        uint8_t pcb_verison = DetectPcbVersion();
        if (!imu_diag_isolation_) { InitializeCharge(); }
        if (!imu_diag_isolation_) { InitializeCst816sTouchPad(); }

        InitializeSpi();
        Initializest77916Display(pcb_verison);
        InitializeButtons();
        
        esp_timer_create_args_t doa_timer_args = {
            .callback = [](void* arg){
                auto& board = (EspS3Cat&)Board::GetInstance();
                auto* codec = board.GetAudioCodec();
                ESP_LOGI(TAG, "Starting DOA task");
                xTaskCreatePinnedToCore(SoundSourceLocator::TaskEntry, "doa_base", 16 * 1024, new SoundSourceLocator(codec), 4, NULL, 1);
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "doa_start",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&doa_timer_args, &doa_start_timer_);
        if (doa_enabled_) {
            esp_timer_start_once(doa_start_timer_, 5000000);
        }

        
        if (BOARD_ENABLE_IMU) {
            xTaskCreatePinnedToCore(imu_task, "imu_task", 4 * 1024, this, 4, NULL, 0);
        } else {
            ESP_LOGW(TAG, "IMU disabled by config (BOARD_ENABLE_IMU=0), skip imu_task");
        }
    }

    virtual AudioCodec* GetAudioCodec() override
    {
        static EchoEarAudioCodec audio_codec(
            i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR,
            AUDIO_CODEC_ES7210_ADDR);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override
    {
        return display_;
    }

    Cst816s* GetTouchpad()
    {
        return cst816s_;
    }

    virtual Backlight* GetBacklight() override
    {
        return backlight_;
    }
};

DECLARE_BOARD(EspS3Cat);

