#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <esp_log.h>
#include "i2c_device.h"
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <wifi_station.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_spd2010.h>
#include <esp_timer.h>
#include "esp_io_expander_tca9554.h"
#include "lcd_display.h"
#include <iot_button.h>
#include "websocket_control_server.h"
#include "touch_element/touch_button.h"  // 添加触摸按钮头文件

#include "esp32_camera.h"

#define TAG "blue_waveshare_lcd_1_46"

extern void InitializeOttoController();
extern void touch_main(void);  // 声明touch_main函数

// 在waveshare_lcd_1_46类之前添加新的显示类
class CustomLcdDisplay : public SpiLcdDisplay {
public:
    static void rounder_event_cb(lv_event_t * e) {
        lv_area_t * area = (lv_area_t *)lv_event_get_param(e);
        uint16_t x1 = area->x1;
        uint16_t x2 = area->x2;

        area->x1 = (x1 >> 2) << 2;          // round the start of coordinate down to the nearest 4M number
        area->x2 = ((x2 >> 2) << 2) + 3;    // round the end of coordinate up to the nearest 4N+3 number
    }

    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle, 
                    esp_lcd_panel_handle_t panel_handle,
                    int width,
                    int height,
                    int offset_x,
                    int offset_y,
                    bool mirror_x,
                    bool mirror_y,
                    bool swap_xy) 
        : SpiLcdDisplay(io_handle, panel_handle,
                    width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
        DisplayLockGuard lock(this);
        lv_display_add_event_cb(display_, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    }
};

class CustomBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    esp_io_expander_handle_t io_expander = NULL;
    LcdDisplay* display_;
    button_handle_t boot_btn, pwr_btn;
    button_driver_t* boot_btn_driver_ = nullptr;
    button_driver_t* pwr_btn_driver_ = nullptr;
    static CustomBoard* instance_;

    WebSocketControlServer* ws_control_server_;
    HardwareConfig hw_config_;
    AudioCodec* audio_codec_;
    Esp32Camera *camera_;
    bool has_camera_;

    bool DetectHardwareVersion() {
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_2_BIT,
            .timer_num = LEDC_TIMER,
            .freq_hz = CAMERA_XCLK_FREQ,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        esp_err_t ret = ledc_timer_config(&ledc_timer);
        if (ret != ESP_OK) {
            return false;
        }
        
        ledc_channel_config_t ledc_channel = {
            .gpio_num = CAMERA_XCLK,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 2,
            .hpoint = 0,
        };
        ret = ledc_channel_config(&ledc_channel);
        if (ret != ESP_OK) {
            return false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = CAMERA_VERSION_CONFIG.i2c_sda_pin,
            .scl_io_num = CAMERA_VERSION_CONFIG.i2c_scl_pin,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        
        ret = i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_);
        if (ret != ESP_OK) {
            ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
            return false;
        }
        const uint8_t camera_addresses[] = {0x30, 0x3C, 0x21, 0x60};
        bool camera_found = false;
        
        for (size_t i = 0; i < sizeof(camera_addresses); i++) {
            uint8_t addr = camera_addresses[i];
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = 100000,
            };
            
            i2c_master_dev_handle_t dev_handle;
            ret = i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev_handle);
            if (ret == ESP_OK) {
                uint8_t reg_addr = 0x0A;
                uint8_t data[2];
                ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, 2, 200);
                if (ret == ESP_OK) {
                    camera_found = true;
                    i2c_master_bus_rm_device(dev_handle);
                    break;
                }
                i2c_master_bus_rm_device(dev_handle);
            }
        }
        
        if (!camera_found) {
            i2c_del_master_bus(i2c_bus_);
            i2c_bus_ = nullptr;
            ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
        }
        return camera_found;
    }
    

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = I2C_SDA_IO,
            .scl_io_num = I2C_SCL_IO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }
    
    void InitializeTca9554(void) {
        esp_err_t ret = esp_io_expander_new_i2c_tca9554(i2c_bus_, I2C_ADDRESS, &io_expander);
        if(ret != ESP_OK)
            ESP_LOGE(TAG, "TCA9554 create returned error");        

        // uint32_t input_level_mask = 0;
        // ret = esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, IO_EXPANDER_INPUT);               // 设置引脚 EXIO0 和 EXIO1 模式为输入 
        // ret = esp_io_expander_get_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, &input_level_mask);             // 获取引脚 EXIO0 和 EXIO1 的电平状态,存放在 input_level_mask 中

        // ret = esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, IO_EXPANDER_OUTPUT);              // 设置引脚 EXIO2 和 EXIO3 模式为输出
        // ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, 1);                             // 将引脚电平设置为 1
        // ret = esp_io_expander_print_state(io_expander);                                                                             // 打印引脚状态

        ret = esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, IO_EXPANDER_OUTPUT);                 // 设置引脚 EXIO0 和 EXIO1 模式为输出
        ESP_ERROR_CHECK(ret);
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, 1);                                // 复位 LCD 与 TouchPad
        ESP_ERROR_CHECK(ret);
        vTaskDelay(pdMS_TO_TICKS(300));
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, 0);                                // 复位 LCD 与 TouchPad
        ESP_ERROR_CHECK(ret);
        vTaskDelay(pdMS_TO_TICKS(300));
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, 1);                                // 复位 LCD 与 TouchPad
        ESP_ERROR_CHECK(ret);
    }

    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize QSPI bus");

        const spi_bus_config_t bus_config = TAIJIPI_SPD2010_PANEL_BUS_QSPI_CONFIG(QSPI_PIN_NUM_LCD_PCLK,
                                                                        QSPI_PIN_NUM_LCD_DATA0,
                                                                        QSPI_PIN_NUM_LCD_DATA1,
                                                                        QSPI_PIN_NUM_LCD_DATA2,
                                                                        QSPI_PIN_NUM_LCD_DATA3,
                                                                        QSPI_LCD_H_RES * 80 * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    void InitializeSpd2010Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGI(TAG, "Install panel IO");
        
        const esp_lcd_panel_io_spi_config_t io_config = SPD2010_PANEL_IO_QSPI_CONFIG(QSPI_PIN_NUM_LCD_CS, NULL, NULL);
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io));

        ESP_LOGI(TAG, "Install SPD2010 panel driver");
        
        spd2010_vendor_config_t vendor_config = {
            .flags = {
                .use_qspi_interface = 1,
            },
        };
        const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = QSPI_PIN_NUM_LCD_RST,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,     // Implemented by LCD command `36h`
            .bits_per_pixel = QSPI_LCD_BIT_PER_PIXEL,    // Implemented by LCD command `3Ah` (16/18)
            .vendor_config = &vendor_config,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_spd2010(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_disp_on_off(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new CustomLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }
 
    void InitializeButtonsCustom() {
        gpio_reset_pin(BOOT_BUTTON_GPIO);                                     
        gpio_set_direction(BOOT_BUTTON_GPIO, GPIO_MODE_INPUT);   
        gpio_reset_pin(PWR_BUTTON_GPIO);                                     
        gpio_set_direction(PWR_BUTTON_GPIO, GPIO_MODE_INPUT);   
        gpio_reset_pin(PWR_Control_PIN);                                     
        gpio_set_direction(PWR_Control_PIN, GPIO_MODE_OUTPUT);     
        // gpio_set_level(PWR_Control_PIN, false);
        gpio_set_level(PWR_Control_PIN, true);
    }

    void InitializeButtons() {
        instance_ = this;
        InitializeButtonsCustom();

        // Boot Button
        button_config_t boot_btn_config = {
            .long_press_time = 2000,
            .short_press_time = 0
        };
        boot_btn_driver_ = (button_driver_t*)calloc(1, sizeof(button_driver_t));
        boot_btn_driver_->enable_power_save = false;
        boot_btn_driver_->get_key_level = [](button_driver_t *button_driver) -> uint8_t {
            return !gpio_get_level(BOOT_BUTTON_GPIO);
        };
        ESP_ERROR_CHECK(iot_button_create(&boot_btn_config, boot_btn_driver_, &boot_btn));
        iot_button_register_cb(boot_btn, BUTTON_SINGLE_CLICK, nullptr, [](void* button_handle, void* usr_data) {
            auto self = static_cast<CustomBoard*>(usr_data);
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                self->ResetWifiConfiguration();
            }
            app.ToggleChatState();
        }, this);
        iot_button_register_cb(boot_btn, BUTTON_LONG_PRESS_START, nullptr, [](void* button_handle, void* usr_data) {
            // 长按无处理
        }, this);

        // Power Button
        button_config_t pwr_btn_config = {
            .long_press_time = 5000,
            .short_press_time = 0
        };
        pwr_btn_driver_ = (button_driver_t*)calloc(1, sizeof(button_driver_t));
        pwr_btn_driver_->enable_power_save = false;
        pwr_btn_driver_->get_key_level = [](button_driver_t *button_driver) -> uint8_t {
            return !gpio_get_level(PWR_BUTTON_GPIO);
        };
        ESP_ERROR_CHECK(iot_button_create(&pwr_btn_config, pwr_btn_driver_, &pwr_btn));
        iot_button_register_cb(pwr_btn, BUTTON_SINGLE_CLICK, nullptr, [](void* button_handle, void* usr_data) {
            // 短按无处理
        }, this);
        iot_button_register_cb(pwr_btn, BUTTON_LONG_PRESS_START, nullptr, [](void* button_handle, void* usr_data) {
            auto self = static_cast<CustomBoard*>(usr_data);
            if(self->GetBacklight()->brightness() > 0) {
                self->GetBacklight()->SetBrightness(0);
                gpio_set_level(PWR_Control_PIN, false);
            }
            else {
                self->GetBacklight()->RestoreBrightness();
                gpio_set_level(PWR_Control_PIN, true);
            }
        }, this);
    }


    
    void InitializeOttoController() {
        ::InitializeOttoController(hw_config_);
    }
    
public:
    const HardwareConfig& GetHardwareConfig() const {
        return hw_config_;
    }
    
private:

    void InitializeWebSocketControlServer() {
        ws_control_server_ = new WebSocketControlServer();
        if (!ws_control_server_->Start(8080)) {
            delete ws_control_server_;
            ws_control_server_ = nullptr;
        }
    }

    void StartNetwork() override {
        WifiBoard::StartNetwork();
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        InitializeWebSocketControlServer();
    }

    bool InitializeCamera() {
        if (!has_camera_ || i2c_bus_ == nullptr) {
            return false;
        }
        
        try {
            static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
                .data_width = CAM_CTLR_DATA_WIDTH_8,
                .data_io = {
                    [0] = CAMERA_D0,
                    [1] = CAMERA_D1,
                    [2] = CAMERA_D2,
                    [3] = CAMERA_D3,
                    [4] = CAMERA_D4,
                    [5] = CAMERA_D5,
                    [6] = CAMERA_D6,
                    [7] = CAMERA_D7,
                },
                .vsync_io = CAMERA_VSYNC,
                .de_io = CAMERA_HSYNC,
                .pclk_io = CAMERA_PCLK,
                .xclk_io = CAMERA_XCLK,
            };

            esp_video_init_sccb_config_t sccb_config = {
                .init_sccb = false,
                .i2c_handle = i2c_bus_,
                .freq = 100000,
            };

            esp_video_init_dvp_config_t dvp_config = {
                .sccb_config = sccb_config,
                .reset_pin = CAMERA_RESET,
                .pwdn_pin = CAMERA_PWDN,
                .dvp_pin = dvp_pin_config,
                .xclk_freq = CAMERA_XCLK_FREQ,
            };

            esp_video_init_config_t video_config = {
                .dvp = &dvp_config,
            };

            camera_ = new Esp32Camera(video_config);
            camera_->SetVFlip(true);
            return true;
        } catch (...) {
            camera_ = nullptr;
            return false;
        }
    }


public:
    OttoRobot() : boot_button_(BOOT_BUTTON_GPIO),
                  audio_codec_(nullptr),
                  i2c_bus_(nullptr),
                  camera_(nullptr),
                  has_camera_(false) {
        
        has_camera_ = DetectHardwareVersion();
        
        if (has_camera_) 
            hw_config_ = CAMERA_VERSION_CONFIG;
        else 
            hw_config_ = NON_CAMERA_VERSION_CONFIG;
        
        
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializePowerManager();
        InitializeAudioCodec();
        
        if (has_camera_) {
            if (!InitializeCamera()) {
                has_camera_ = false;
            }
        }
        
        InitializeOttoController();
        ws_control_server_ = nullptr;
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec *GetAudioCodec() override {
        return audio_codec_;
    }

    virtual Display* GetDisplay() override { 
        return display_; 
    }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight* backlight = nullptr;
        if (backlight == nullptr) {
            backlight = new PwmBacklight(hw_config_.display_backlight_pin, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        }
        return backlight;
    }
    
    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        charging = power_manager_->IsCharging();
        discharging = !charging;
        level = power_manager_->GetBatteryLevel();
        return true;
    }

    virtual Camera *GetCamera() override { 
        return has_camera_ ? camera_ : nullptr; 
    }
};



DECLARE_BOARD(CustomBoard);

CustomBoard* CustomBoard::instance_ = nullptr;
