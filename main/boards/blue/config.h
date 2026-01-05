#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define RIGHT_LEG_PIN GPIO_NUM_13
#define RIGHT_FOOT_PIN GPIO_NUM_1
#define LEFT_LEG_PIN GPIO_NUM_3
#define LEFT_FOOT_PIN GPIO_NUM_14
#define LEFT_HAND_PIN GPIO_NUM_16
#define RIGHT_HAND_PIN GPIO_NUM_17

#define BLUE_ROBOT_VERSION "1.0.0"

/////////////////
#include <driver/spi_master.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define PWR_BUTTON_GPIO         GPIO_NUM_6
#define PWR_Control_PIN         GPIO_NUM_7

#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_2
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_15
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_39
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_47
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_48
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_38

#define I2C_SCL_IO          GPIO_NUM_10       
#define I2C_SDA_IO          GPIO_NUM_11        


#define I2C_ADDRESS         ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000

#define DISPLAY_WIDTH       412
#define DISPLAY_HEIGHT      412
#define DISPLAY_MIRROR_X    false
#define DISPLAY_MIRROR_Y    false
#define DISPLAY_SWAP_XY     false

#define QSPI_LCD_H_RES           (412)
#define QSPI_LCD_V_RES           (412)
#define QSPI_LCD_BIT_PER_PIXEL   (16)

#define QSPI_LCD_HOST           SPI2_HOST
#define QSPI_PIN_NUM_LCD_PCLK   GPIO_NUM_40
#define QSPI_PIN_NUM_LCD_CS     GPIO_NUM_21
#define QSPI_PIN_NUM_LCD_DATA0  GPIO_NUM_46
#define QSPI_PIN_NUM_LCD_DATA1  GPIO_NUM_45
#define QSPI_PIN_NUM_LCD_DATA2  GPIO_NUM_42
#define QSPI_PIN_NUM_LCD_DATA3  GPIO_NUM_41
#define QSPI_PIN_NUM_LCD_RST    GPIO_NUM_NC
#define QSPI_PIN_NUM_LCD_BL     GPIO_NUM_5

#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0

#define TP_PORT          (I2C_NUM_1)
#define TP_PIN_NUM_SDA   (I2C_SDA_IO)
#define TP_PIN_NUM_SCL   (I2C_SCL_IO)
#define TP_PIN_NUM_RST   (GPIO_NUM_NC)
#define TP_PIN_NUM_INT   (GPIO_NUM_4)

#define DISPLAY_BACKLIGHT_PIN           QSPI_PIN_NUM_LCD_BL
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

#define TAIJIPI_SPD2010_PANEL_BUS_QSPI_CONFIG(sclk, d0, d1, d2, d3, max_trans_sz) \
    {                                                                             \
        .data0_io_num = d0,                                                       \
        .data1_io_num = d1,                                                       \
        .sclk_io_num = sclk,                                                      \
        .data2_io_num = d2,                                                       \
        .data3_io_num = d3,                                                       \
        .max_transfer_sz = max_trans_sz,                                          \
    }

struct HardwareConfig {
    gpio_num_t power_charge_detect_pin;
    /*
    adc_unit_t power_adc_unit;
    adc_channel_t power_adc_channel;
    */

    gpio_num_t right_leg_pin;
    gpio_num_t right_foot_pin;
    gpio_num_t left_leg_pin;
    gpio_num_t left_foot_pin;
    gpio_num_t left_hand_pin;
    gpio_num_t right_hand_pin;
    
    int audio_input_sample_rate;
    int audio_output_sample_rate;
    bool audio_use_simplex;
    
    gpio_num_t audio_i2s_gpio_ws;
    gpio_num_t audio_i2s_gpio_bclk;
    gpio_num_t audio_i2s_gpio_din;
    gpio_num_t audio_i2s_gpio_dout;
    
    gpio_num_t audio_i2s_mic_gpio_ws;
    gpio_num_t audio_i2s_mic_gpio_sck;
    gpio_num_t audio_i2s_mic_gpio_din;
    gpio_num_t audio_i2s_spk_gpio_dout;
    gpio_num_t audio_i2s_spk_gpio_bclk;
    gpio_num_t audio_i2s_spk_gpio_lrck;
    
    gpio_num_t display_backlight_pin;
    gpio_num_t display_mosi_pin;
    gpio_num_t display_clk_pin;
    gpio_num_t display_dc_pin;
    gpio_num_t display_rst_pin;
    gpio_num_t display_cs_pin;
    
    gpio_num_t i2c_sda_pin;
    gpio_num_t i2c_scl_pin;
};

constexpr HardwareConfig CAMERA_VERSION_CONFIG = {
    /*
    .power_charge_detect_pin = GPIO_NUM_NC,
    .power_adc_unit = ADC_UNIT_1,
    .power_adc_channel = ADC_CHANNEL_1,
    */

    .right_leg_pin = RIGHT_LEG_PIN,
    .right_foot_pin = RIGHT_FOOT_PIN,
    .left_leg_pin = LEFT_LEG_PIN,
    .left_foot_pin = LEFT_FOOT_PIN,
    .left_hand_pin = LEFT_HAND_PIN,
    .right_hand_pin = RIGHT_HAND_PIN,
    
    /*
    .audio_input_sample_rate = 16000,
    .audio_output_sample_rate = 16000,
    .audio_use_simplex = false,
    
    .audio_i2s_gpio_ws = GPIO_NUM_40,
    .audio_i2s_gpio_bclk = GPIO_NUM_42,
    .audio_i2s_gpio_din = GPIO_NUM_41,
    .audio_i2s_gpio_dout = GPIO_NUM_39,
    
    .audio_i2s_mic_gpio_ws = GPIO_NUM_NC,
    .audio_i2s_mic_gpio_sck = GPIO_NUM_NC,
    .audio_i2s_mic_gpio_din = GPIO_NUM_NC,
    .audio_i2s_spk_gpio_dout = GPIO_NUM_NC,
    .audio_i2s_spk_gpio_bclk = GPIO_NUM_NC,
    .audio_i2s_spk_gpio_lrck = GPIO_NUM_NC,
    
    .display_backlight_pin = GPIO_NUM_38,
    .display_mosi_pin = GPIO_NUM_45,
    .display_clk_pin = GPIO_NUM_48,
    .display_dc_pin = GPIO_NUM_47,
    .display_rst_pin = GPIO_NUM_1,
    .display_cs_pin = GPIO_NUM_NC,
    
    .i2c_sda_pin = GPIO_NUM_15,
    .i2c_scl_pin = GPIO_NUM_16,
    */
};

constexpr HardwareConfig NON_CAMERA_VERSION_CONFIG = {
    /*
    .power_charge_detect_pin = GPIO_NUM_21,
    .power_adc_unit = ADC_UNIT_2,
    .power_adc_channel = ADC_CHANNEL_3,
    */

    .right_leg_pin = GPIO_NUM_39,
    .right_foot_pin = GPIO_NUM_38,
    .left_leg_pin = GPIO_NUM_17,
    .left_foot_pin = GPIO_NUM_18,
    .left_hand_pin = GPIO_NUM_8,
    .right_hand_pin = GPIO_NUM_12,
    
    /*
    .audio_input_sample_rate = 16000,
    .audio_output_sample_rate = 24000,
    .audio_use_simplex = true,
    
    .audio_i2s_gpio_ws = GPIO_NUM_NC,
    .audio_i2s_gpio_bclk = GPIO_NUM_NC,
    .audio_i2s_gpio_din = GPIO_NUM_NC,
    .audio_i2s_gpio_dout = GPIO_NUM_NC,
    
    .audio_i2s_mic_gpio_ws = GPIO_NUM_4,
    .audio_i2s_mic_gpio_sck = GPIO_NUM_5,
    .audio_i2s_mic_gpio_din = GPIO_NUM_6,
    .audio_i2s_spk_gpio_dout = GPIO_NUM_7,
    .audio_i2s_spk_gpio_bclk = GPIO_NUM_15,
    .audio_i2s_spk_gpio_lrck = GPIO_NUM_16,
    
    .display_backlight_pin = GPIO_NUM_3,
    .display_mosi_pin = GPIO_NUM_10,
    .display_clk_pin = GPIO_NUM_9,
    .display_dc_pin = GPIO_NUM_46,
    .display_rst_pin = GPIO_NUM_11,
    .display_cs_pin = GPIO_NUM_12,
    
    .i2c_sda_pin = GPIO_NUM_NC,
    .i2c_scl_pin = GPIO_NUM_NC,
    */
};

#define CAMERA_XCLK (GPIO_NUM_3)
#define CAMERA_PCLK (GPIO_NUM_10)
#define CAMERA_VSYNC (GPIO_NUM_17)
#define CAMERA_HSYNC (GPIO_NUM_18)
#define CAMERA_D0 (GPIO_NUM_12)
#define CAMERA_D1 (GPIO_NUM_14)
#define CAMERA_D2 (GPIO_NUM_21)
#define CAMERA_D3 (GPIO_NUM_13)
#define CAMERA_D4 (GPIO_NUM_11)
#define CAMERA_D5 (GPIO_NUM_9)
#define CAMERA_D6 (GPIO_NUM_46)
#define CAMERA_D7 (GPIO_NUM_8)
#define CAMERA_PWDN (GPIO_NUM_NC)
#define CAMERA_RESET (GPIO_NUM_NC)
#define CAMERA_XCLK_FREQ (16000000)
#define LEDC_TIMER (LEDC_TIMER_0)
#define LEDC_CHANNEL (LEDC_CHANNEL_0)



/*
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR true
#define DISPLAY_RGB_ORDER LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 3

#define BOOT_BUTTON_GPIO GPIO_NUM_0
*/
#endif // _BOARD_CONFIG_H_