#include <stdio.h>
#include "bsp/device.h"
#include "bsp/display.h"
#include "bsp/input.h"
#include "bsp/led.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"
#include "esp_log.h"
#include "hal/lcd_types.h"
#include "nvs_flash.h"
#include "pax_fonts.h"
#include "pax_gfx.h"
#include "pax_text.h"
#include "portmacro.h"

#include "bmi270.h"
#include "bmi270_interface.h"
#include "bsp/i2c.h"

#include "freertos/FreeRTOS.h"
#include "esp_random.h"
// #include "freeRTOS\task.h"

#include "bsp/audio.h"
#include "driver/i2s_std.h"

//Audio stuff
#define PLAYBACK_BUFFER_SIZE_BYTES  (2048)
#define PLAYBACK_SAMPLE_RATE        (44100) //In Hz
#define PLAYBACK_BITS_PER_SAMPLE    (I2S_DATA_BIT_WIDTH_16BIT)
#define SINE_WAVE_FREQUENCY         (440) // A4 note
#define AMPLITUDE                   (INT16_MAX / 2) // Reduce amplitude to avoid clipping
#define TAG_PLAYBACK                "PLAYBACK"

//IMU stuff
#define TAG_IMU                     "IMU"
#define GRAVITY_EARTH (9.80665f)

void run_leds(void *parameter);

// Constants
static char const TAG[] = "main";

// Global variables
static size_t                       display_h_res        = 0;
static size_t                       display_v_res        = 0;
static lcd_color_rgb_pixel_format_t display_color_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
static lcd_rgb_data_endian_t        display_data_endian  = LCD_RGB_DATA_ENDIAN_LITTLE;
static pax_buf_t                    fb                   = {0};
static QueueHandle_t                input_event_queue    = NULL;

//Accelerometer stuff
float g_gyro_x = 0.0f;
float g_gyro_y = 0.0f;
struct imu_data {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

//Game stuff
#define HOLES 10
uint32_t holes_pos[HOLES*2];
uint32_t food_x=999999;
uint32_t food_y=999999;
uint32_t score = 0;
// srand(time(NULL));

//GRB
uint8_t led_data[] = {
    0xfb, 0x5b, 0xcf, 0xab, 0xf5, 0xb9, 0xff, 0xff, 0xff, 0xab, 0xf5, 0xb9,  0xfb, 0x5b, 0xcf, 0x00, 0x00, 0x00,
};

#if defined(CONFIG_BSP_TARGET_KAMI)
static pax_col_t palette[] = {0xffffffff, 0xff000000, 0xffff0000};  // white, black, red
#endif

void init_holes(uint32_t max_x, uint32_t max_y) {
    int i;
    for (i = 0; i < HOLES*2; i+=2) {
        holes_pos[i] = esp_random()%max_x;
        holes_pos[i+1] = esp_random()%max_y;
    }
}

void get_food_pos(uint32_t max_x, uint32_t max_y) {
    food_x = esp_random() % max_x;
    food_y = esp_random()%max_y;
}

static int8_t set_gyro_config(struct bmi2_dev* dev) {
    int8_t                  rslt;
    struct bmi2_sens_config config;
    config.type = BMI2_GYRO;
    rslt        = bmi2_get_sensor_config(&config, 1, dev);
    bmi2_error_codes_print_result(rslt);
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT2, dev);
    bmi2_error_codes_print_result(rslt);
    if (rslt == BMI2_OK) {
        config.cfg.gyr.odr         = BMI2_GYR_ODR_100HZ;
        config.cfg.gyr.range       = BMI2_GYR_RANGE_2000;
        config.cfg.gyr.bwp         = BMI2_GYR_NORMAL_MODE;
        config.cfg.gyr.noise_perf  = BMI2_POWER_OPT_MODE;
        config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
        rslt                       = bmi2_set_sensor_config(&config, 1, dev);
    }
    return rslt;
}

static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width) {
    double power      = 2;
    float  half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (dps / (half_scale)) * (val);
}

static int8_t set_accel_config(struct bmi2_dev* bmi) {
    int8_t rslt;

    struct bmi2_sens_config config;
    config.type = BMI2_ACCEL;

    rslt = bmi2_get_sensor_config(&config, 1, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK) {
        config.cfg.acc.odr         = BMI2_ACC_ODR_200HZ;
        config.cfg.acc.range       = BMI2_ACC_RANGE_2G;
        config.cfg.acc.bwp         = BMI2_ACC_NORMAL_AVG4;
        config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        rslt = bmi2_set_sensor_config(&config, 1, bmi);
        bmi2_error_codes_print_result(rslt);

        rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width) {
    double power      = 2;
    float  half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

void blit(void) {
    bsp_display_blit(0, 0, display_h_res, display_v_res, pax_buf_get_pixels(&fb));
}

void clear_screen() {
    pax_simple_rect(&fb, 0xFFFFFFFF, 0, 0, pax_buf_get_width(&fb), pax_buf_get_height(&fb));
}

struct bmi2_dev init_bmi() {
    i2c_master_bus_handle_t handle;
    SemaphoreHandle_t       semaphore;
    bsp_i2c_primary_bus_get_handle(&handle);
    bsp_i2c_primary_bus_get_semaphore(&semaphore);
    bmi2_set_i2c_configuration(handle, 0x68, semaphore);

    int8_t                rslt;
    struct bmi2_dev       bmi;
    struct bmi2_sens_data sensor_data;
    uint8_t               sensor_list[] = {BMI2_GYRO, BMI2_ACCEL};

    memset(&bmi, 0, sizeof(struct bmi2_dev));
    memset(&sensor_data, 0, sizeof(struct bmi2_sens_data));

    printf("Interface init...\r\n");
    rslt = bmi2_interface_init(&bmi, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        printf("Failed to initialize interface\r\n");
        // return NULL;
    }

    printf("Init...\r\n");
    rslt = bmi270_init(&bmi);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        printf("Failed to initialize chip\r\n");
        // return NULL;
    }

    printf("Config gyro...\r\n");
    rslt = set_gyro_config(&bmi);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        printf("Failed to set gyro config\r\n");
        // return NULL;
    }

    printf("Config accel...\r\n");
    rslt = set_accel_config(&bmi);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        printf("Failed to set accel config\r\n");
        // return NULL;
    }

    printf("Enable sensors...\r\n");
    rslt = bmi2_sensor_enable(sensor_list, sizeof(sensor_list), &bmi);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        printf("Failed to enable\r\n");
        // return NULL;
    }

    printf("Ready to get data\n");

    return bmi;
}

struct imu_data get_imu_data(struct bmi2_dev bmi) {
    struct bmi2_sens_data sensor_data;
    int8_t                rslt;
    struct imu_data       data;

    rslt = bmi2_get_sensor_data(&sensor_data, &bmi);
    printf("Got data or an error %d\n",rslt);
    bmi2_error_codes_print_result(rslt);

    if ((rslt == BMI2_OK) && (sensor_data.status & BMI2_DRDY_GYR) && (sensor_data.status & BMI2_DRDY_ACC)) {
        /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
        float gyro_x = lsb_to_dps(sensor_data.gyr.x, (float)2000, bmi.resolution);
        float gyro_y = lsb_to_dps(sensor_data.gyr.y, (float)2000, bmi.resolution);
        float gyro_z = lsb_to_dps(sensor_data.gyr.z, (float)2000, bmi.resolution);

        /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
        float accel_x = lsb_to_mps2(sensor_data.acc.x, (float)2, bmi.resolution);
        float accel_y = lsb_to_mps2(sensor_data.acc.y, (float)2, bmi.resolution);
        float accel_z = lsb_to_mps2(sensor_data.acc.z, (float)2, bmi.resolution);

        // printf("A: %4.2f, %4.2f, %4.2f  G: %4.2f, %4.2f, %4.2f\n", accel_x, accel_y, accel_z, gyro_x, gyro_y,
        //         gyro_z);

        data.accel_x = accel_x;
        data.accel_y = accel_y;
        data.accel_z = accel_z;
        data.gyro_x = gyro_x;
        data.gyro_y = gyro_y;
        data.gyro_z = gyro_z;
    }

    return data;
}

void test_bmi270(void *params) {
    i2c_master_bus_handle_t handle;
    SemaphoreHandle_t       semaphore;
    bsp_i2c_primary_bus_get_handle(&handle);
    bsp_i2c_primary_bus_get_semaphore(&semaphore);
    bmi2_set_i2c_configuration(handle, 0x68, semaphore);

    int8_t                rslt;
    struct bmi2_dev       bmi;
    struct bmi2_sens_data sensor_data;
    uint8_t               sensor_list[] = {BMI2_GYRO, BMI2_ACCEL};

    memset(&bmi, 0, sizeof(struct bmi2_dev));
    memset(&sensor_data, 0, sizeof(struct bmi2_sens_data));

    printf("Interface init...\r\n");
    rslt = bmi2_interface_init(&bmi, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        printf("Failed to initialize interface\r\n");
        return;
    }

    printf("Init...\r\n");
    rslt = bmi270_init(&bmi);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        printf("Failed to initialize chip\r\n");
        return;
    }

    printf("Config gyro...\r\n");
    rslt = set_gyro_config(&bmi);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        printf("Failed to set gyro config\r\n");
        return;
    }

    printf("Config accel...\r\n");
    rslt = set_accel_config(&bmi);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        printf("Failed to set accel config\r\n");
        return;
    }

    printf("Enable sensors...\r\n");
    rslt = bmi2_sensor_enable(sensor_list, sizeof(sensor_list), &bmi);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        printf("Failed to enable\r\n");
        return;
    }

    printf("Ready to get data\n");

    while (true) {
        rslt = bmi2_get_sensor_data(&sensor_data, &bmi);
        printf("Got data or an error %d\n",rslt);
        bmi2_error_codes_print_result(rslt);

        if ((rslt == BMI2_OK) && (sensor_data.status & BMI2_DRDY_GYR) && (sensor_data.status & BMI2_DRDY_ACC)) {
            /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
            float gyro_x = lsb_to_dps(sensor_data.gyr.x, (float)2000, bmi.resolution);
            float gyro_y = lsb_to_dps(sensor_data.gyr.y, (float)2000, bmi.resolution);
            float gyro_z = lsb_to_dps(sensor_data.gyr.z, (float)2000, bmi.resolution);

            g_gyro_x = gyro_x;
            g_gyro_y = gyro_y;

            /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
            float accel_x = lsb_to_mps2(sensor_data.acc.x, (float)2, bmi.resolution);
            float accel_y = lsb_to_mps2(sensor_data.acc.y, (float)2, bmi.resolution);
            float accel_z = lsb_to_mps2(sensor_data.acc.z, (float)2, bmi.resolution);

            printf("A: %4.2f, %4.2f, %4.2f  G: %4.2f, %4.2f, %4.2f\n", accel_x, accel_y, accel_z, gyro_x, gyro_y,
                   gyro_z);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void run_leds(void *parameter) {
    while(1) {
        bsp_led_write(led_data, sizeof(led_data));
        int i = 0;
        int j;
        uint8_t temp;
        for (j = 0; j < 3; j++) {
            temp = led_data[0];
            for (i = 0; i < 17; i++) {
                led_data[i] = led_data[i+1];
            }
            led_data[17] = temp;
        }
        // for (int j = 0; j < 3; j++) {
        //     led_data[i] = temp;
        // }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

//This function is based on: https://circuitlabs.net/i2s-audio-codec-integration-with-esp-idf/
void playback_sine_wave_task(void *arg) {
    i2s_chan_handle_t my_i2s_handle = NULL;
    bsp_audio_initialize((uint32_t)PLAYBACK_SAMPLE_RATE); //Initalize the handle using the wanted sample rate
    bsp_audio_set_volume(60);  //Setting the volume in percentage%
    bsp_audio_set_amplifier(true); //Enable speaker
    bsp_audio_get_i2s_handle(&my_i2s_handle); //Get the prepared handle

    if (my_i2s_handle == NULL) {
        ESP_LOGE(TAG_PLAYBACK, "Handle is null. Quitting....");
        return; //The application (more specific, this task) will crash if the handle is null and there is returned
    }

    uint8_t *tx_buffer = (uint8_t *)malloc(PLAYBACK_BUFFER_SIZE_BYTES);
    if (!tx_buffer) {
        ESP_LOGE(TAG_PLAYBACK, "Failed to allocate TX buffer");
        vTaskDelete(NULL);
        return; //The application (more specific, this task) will crash if not possible to allocate memeory and there is returned
    }

    ESP_LOGI(TAG_PLAYBACK, "Starting sine wave playback...");
    size_t bytes_written = 0;
    double time_step = 1.0 / PLAYBACK_SAMPLE_RATE; //t=1/f
    double current_time = 0;
    int16_t *samples16 = (int16_t *)tx_buffer;

    while (1) {
        int num_frames = PLAYBACK_BUFFER_SIZE_BYTES / ( (PLAYBACK_BITS_PER_SAMPLE / 8) * 2); // 2 channels for stereo

        for (int i = 0; i < num_frames; i++) {
            //Get the current value based on where on the sine curve we are to the given time
            int16_t sample_val = (int16_t)(AMPLITUDE * sin(2 * M_PI * SINE_WAVE_FREQUENCY * current_time));
            samples16[i * 2 + 0] = sample_val; // Left channel
            samples16[i * 2 + 1] = sample_val; // Right channel (mono sound on stereo)
            current_time += time_step;
        }

        //Write data (the frames made in the for loop) to the channel. See 
        //https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2s.html#_CPPv417i2s_channel_write17i2s_chan_handle_tPKv6size_tP6size_t8uint32_t 
        //for more
        esp_err_t ret = i2s_channel_write(my_i2s_handle, tx_buffer, PLAYBACK_BUFFER_SIZE_BYTES, &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_PLAYBACK, "I2S write error: %s", esp_err_to_name(ret));
        } else if (bytes_written < PLAYBACK_BUFFER_SIZE_BYTES) {
            ESP_LOGW(TAG_PLAYBACK, "I2S write underrun: wrote %d of %d bytes", bytes_written, PLAYBACK_BUFFER_SIZE_BYTES);
        }
        // vTaskDelay(pdMS_TO_TICKS(10)); //Just sleep to get the rest of the UI to function Not needed if everything works
    }
    // free(tx_buffer); // Unreachable but you need to clean up your memory if you stop the playback
    // vTaskDelete(NULL); //Delete the task to clean up after ourself
}

/*
TODO game
Spil hvor man skal undgå nogle sorte huller og "spise" nogle point
Score står i hjørnet og bliver løbende opdateret

Lav positioner for huller (maks X huller). Hvis cirklens midtpunkt er inde i en af de sorte huller, så er spillet slut
Lav position for en mad. Når den er spist så lav en ny

Have et array med position på sorte huller.
Have noget mad
*/

void app_main(void) {
    // Start the GPIO interrupt service
    gpio_install_isr_service(0);

    // Initialize the Non Volatile Storage service
    esp_err_t res = nvs_flash_init();
    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        res = nvs_flash_init();
    }
    ESP_ERROR_CHECK(res);

    // Initialize the Board Support Package
    ESP_ERROR_CHECK(bsp_device_initialize());
    bsp_led_initialize();
    //0x47, 0x00, 0xDF, 0x97, 0x5A, 0xEE, 0xD1, 0x4C, 0xE5, 0xCA, 0x68, 0x65, 0x89, 0xEA, 0x14, 0x25, 0xB8, 0x73,
    //GRB
    //LED0: Power
    //LED1: Antenna
    //LED2: Message
    //LED3: Power
    //LED4: A
    //LED5: B
    // uint8_t led_data[] = {
    //     0x47, 0x00, 0xDF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
    // };
    // bsp_led_write(led_data, sizeof(led_data));

    xTaskCreate(run_leds, "running leds", 2048, NULL, 2, NULL);
    
    // xTaskCreate(playback_sine_wave_task, "sine_playback", 4096, NULL, 5, NULL);

    struct bmi2_dev bmi = init_bmi();

    // xTaskCreate(test_bmi270, "Testing BMI270 IMU", 4096, NULL, 2, NULL);

    // Get display parameters and rotation
    res = bsp_display_get_parameters(&display_h_res, &display_v_res, &display_color_format, &display_data_endian);
    ESP_ERROR_CHECK(res);  // Check that the display parameters have been initialized
    bsp_display_rotation_t display_rotation = bsp_display_get_default_rotation();

    // Convert ESP-IDF color format into PAX buffer type
    pax_buf_type_t format = PAX_BUF_24_888RGB;
    switch (display_color_format) {
        case LCD_COLOR_PIXEL_FORMAT_RGB565:
            format = PAX_BUF_16_565RGB;
            break;
        case LCD_COLOR_PIXEL_FORMAT_RGB888:
            format = PAX_BUF_24_888RGB;
            break;
        default:
            break;
    }

    // Convert BSP display rotation format into PAX orientation type
    pax_orientation_t orientation = PAX_O_UPRIGHT;
    switch (display_rotation) {
        case BSP_DISPLAY_ROTATION_90:
            orientation = PAX_O_ROT_CCW;
            break;
        case BSP_DISPLAY_ROTATION_180:
            orientation = PAX_O_ROT_HALF;
            break;
        case BSP_DISPLAY_ROTATION_270:
            orientation = PAX_O_ROT_CW;
            break;
        case BSP_DISPLAY_ROTATION_0:
        default:
            orientation = PAX_O_UPRIGHT;
            break;
    }

        // Initialize graphics stack
#if defined(CONFIG_BSP_TARGET_KAMI)
    format = PAX_BUF_2_PAL;
#endif

    pax_buf_init(&fb, NULL, display_h_res, display_v_res, format);
    pax_buf_reversed(&fb, display_data_endian == LCD_RGB_DATA_ENDIAN_BIG);

#if defined(CONFIG_BSP_TARGET_KAMI)
    fb.palette      = palette;
    fb.palette_size = sizeof(palette) / sizeof(pax_col_t);
#endif

#if defined(CONFIG_BSP_TARGET_KAMI)
#define BLACK 0
#define WHITE 1
#define RED   2
#else
#define BLACK 0xFF000000
#define WHITE 0xFFFFFFFF
#define RED   0xFFFF0000
#define GREEN 0xFF00FF00
#define BLUE 0xFF0000FF
#endif

    pax_buf_set_orientation(&fb, orientation);

    // Get input event queue from BSP
    ESP_ERROR_CHECK(bsp_input_get_queue(&input_event_queue));

    ESP_LOGW(TAG, "Hello world!");

    pax_background(&fb, WHITE);
    char score_text[20];
    sprintf(score_text, "Score: %d", ((int)score));
    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 0, score_text);

    float midpoint_x = pax_buf_get_width(&fb) / 2.0;  // Middle of the screen horizontally.
    float midpoint_y = pax_buf_get_height(&fb) / 2.0; // Middle of the screen vertically.
    float radius     = 5;                             // Nice, big circle.
    pax_simple_circle(&fb, pax_col_rgb(255, 0, 0), midpoint_x, midpoint_y, radius);

    blit();

    struct imu_data data;

    init_holes(pax_buf_get_width(&fb), pax_buf_get_height(&fb));
    get_food_pos(pax_buf_get_width(&fb), pax_buf_get_height(&fb));

    while (1) {
        //clear_screen();
        bsp_input_event_t event;
        pax_simple_circle(&fb, WHITE, midpoint_x, midpoint_y, radius);

        pax_simple_tri(&fb, GREEN, food_x, food_y-10, food_x-10, food_y+10, food_x+10, food_y+10);
        for (int j = 0; j < HOLES*2; j+=2) {
            pax_simple_rect(&fb, BLUE, holes_pos[j]-10,holes_pos[j+1]-10,20,20);
        }

        if (midpoint_x+radius > pax_buf_get_width(&fb)) {
            midpoint_x = pax_buf_get_width(&fb)-radius;
        }

        if (midpoint_x-radius < 0) {
            midpoint_x = 0+radius;
        }

        if (midpoint_y+radius > pax_buf_get_height(&fb)) {
            midpoint_y = pax_buf_get_height(&fb)-radius;
        }

        if (midpoint_y-radius < 0) {
            midpoint_y = 0+radius;
        }

        data = get_imu_data(bmi);

        if (data.accel_y>1) {
            midpoint_x+=10;
        }
        if (data.accel_y<-1) {
            midpoint_x-=10;
        }
        if (data.accel_x>1) {
            midpoint_y-=10;
        }
        if (data.accel_x<-1) {
            midpoint_y+=10;
        }
        pax_simple_circle(&fb, pax_col_rgb(255, 0, 0), midpoint_x, midpoint_y, radius);
        blit();
        vTaskDelay(pdMS_TO_TICKS(100));
        /**/
        if (xQueueReceive(input_event_queue, &event, 0/*portMAX_DELAY*/) == pdTRUE) {
            switch (event.type) {
                case INPUT_EVENT_TYPE_KEYBOARD: {
                    if (event.args_keyboard.ascii != '\b' ||
                        event.args_keyboard.ascii != '\t') {  // Ignore backspace & tab keyboard events
                        ESP_LOGI(TAG, "Keyboard event %c (%02x) %s", event.args_keyboard.ascii,
                                 (uint8_t)event.args_keyboard.ascii, event.args_keyboard.utf8);
                        pax_simple_rect(&fb, WHITE, 0, 0, pax_buf_get_width(&fb), 72);
                        pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 0, "Keyboard event");
                        char text[64];
                        snprintf(text, sizeof(text), "ASCII:     %c (0x%02x)", event.args_keyboard.ascii,
                                 (uint8_t)event.args_keyboard.ascii);
                        pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 18, text);
                        snprintf(text, sizeof(text), "UTF-8:     %s", event.args_keyboard.utf8);
                        pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 36, text);
                        snprintf(text, sizeof(text), "Modifiers: 0x%0" PRIX32, event.args_keyboard.modifiers);
                        pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 54, text);
                        blit();
                    }
                    break;
                }
                case INPUT_EVENT_TYPE_NAVIGATION: {
                    ESP_LOGI(TAG, "Navigation event %0" PRIX32 ": %s", (uint32_t)event.args_navigation.key,
                             event.args_navigation.state ? "pressed" : "released");

                    if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_F1) {
                        bsp_input_set_backlight_brightness(0);
                    }
                    if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_F2) {
                        bsp_input_set_backlight_brightness(100);
                    }

                    if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_ESC) {
                        bsp_device_restart_to_launcher();
                    }
                    if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_LEFT && event.args_navigation.state) {
                        //pax_simple_rect(&fb, WHITE, midpoint_x-25, midpoint_y-25, midpoint_x+25, midpoint_y+25);
                        pax_simple_circle(&fb, WHITE, midpoint_x, midpoint_y, radius);
                        midpoint_x -= 10;
                        pax_simple_circle(&fb, pax_col_rgb(255, 0, 0), midpoint_x, midpoint_y, radius);
                    }

                    if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_DOWN && event.args_navigation.state) {
                        //pax_simple_rect(&fb, WHITE, midpoint_x-25, midpoint_y-25, midpoint_x+25, midpoint_y+25);
                        pax_simple_circle(&fb, WHITE, midpoint_x, midpoint_y, radius);
                        midpoint_y += 10;
                        pax_simple_circle(&fb, pax_col_rgb(255, 0, 0), midpoint_x, midpoint_y, radius);
                    }
                    if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_RIGHT && event.args_navigation.state) {
                        //pax_simple_rect(&fb, WHITE, midpoint_x-25, midpoint_y-25, midpoint_x+25, midpoint_y+25);
                        pax_simple_circle(&fb, WHITE, midpoint_x, midpoint_y, radius);
                        midpoint_x += 10;
                        pax_simple_circle(&fb, pax_col_rgb(255, 0, 0), midpoint_x, midpoint_y, radius);
                    }

                    if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_UP && event.args_navigation.state) {
                        //pax_simple_rect(&fb, WHITE, midpoint_x-25, midpoint_y-25, midpoint_x+25, midpoint_y+25);
                        pax_simple_circle(&fb, WHITE, midpoint_x, midpoint_y, radius);
                        midpoint_y -= 10;
                        pax_simple_circle(&fb, pax_col_rgb(255, 0, 0), midpoint_x, midpoint_y, radius);
                    }

                    pax_simple_rect(&fb, WHITE, 0, 100, pax_buf_get_width(&fb), 72);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 100 + 0, "Navigation event");
                    char text[64];
                    snprintf(text, sizeof(text), "Key:       0x%0" PRIX32, (uint32_t)event.args_navigation.key);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 100 + 18, text);
                    snprintf(text, sizeof(text), "State:     %s", event.args_navigation.state ? "pressed" : "released");
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 100 + 36, text);
                    snprintf(text, sizeof(text), "Modifiers: 0x%0" PRIX32, event.args_navigation.modifiers);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 100 + 54, text);
                    blit();
                    break;
                }
                case INPUT_EVENT_TYPE_ACTION: {
                    ESP_LOGI(TAG, "Action event 0x%0" PRIX32 ": %s", (uint32_t)event.args_action.type,
                             event.args_action.state ? "yes" : "no");
                    pax_simple_rect(&fb, WHITE, 0, 200 + 0, pax_buf_get_width(&fb), 72);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 200 + 0, "Action event");
                    char text[64];
                    snprintf(text, sizeof(text), "Type:      0x%0" PRIX32, (uint32_t)event.args_action.type);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 200 + 36, text);
                    snprintf(text, sizeof(text), "State:     %s", event.args_action.state ? "yes" : "no");
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 200 + 54, text);
                    blit();
                    break;
                }
                case INPUT_EVENT_TYPE_SCANCODE: {
                    ESP_LOGI(TAG, "Scancode event 0x%0" PRIX32, (uint32_t)event.args_scancode.scancode);
                    pax_simple_rect(&fb, WHITE, 0, 300 + 0, pax_buf_get_width(&fb), 72);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 300 + 0, "Scancode event");
                    char text[64];
                    snprintf(text, sizeof(text), "Scancode:  0x%0" PRIX32, (uint32_t)event.args_scancode.scancode);
                    pax_draw_text(&fb, BLACK, pax_font_sky_mono, 16, 0, 300 + 36, text);
                    blit();
                    break;
                }
                default:
                    break;
            }
        }
        /**/
    }
}
