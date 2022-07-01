#include <esp_wifi.h>
//#include <esp_bt.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#include <dht.h>

#include <driver/adc_common.h>

#include "esp_http_client.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc_cal.h"

#include "esp_camera.h"

#include "driver/rtc_io.h"

#include "esp_timer.h"



static const char *TAG = "camera_demo";

/* Used to prevent light sleep during camera read */
static esp_pm_lock_handle_t pm_lock;


/* Upload photos to here */
#define HTTP_HOST "192.168.1.101"
#define HTTP_PORT 8000
#define HTTP_IMAGE_PATH "/image"
#define HTTP_DATA_PATH "/data"
#define HTTP_TIMEOUT_SEC 10

/* DHT-22 sensor */
#define DHT_GPIO 15

/* Number of times to read the DHT sensor when logging data.
 * first read seems to produce a stale value */
#define DHT_NEEDED_READS 1

/* ADC for battery voltage measurement */
#define ADC_GPIO 13
#define ADC_UNIT ADC_UNIT_2
#define ADC_CHANNEL ADC2_CHANNEL_4
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_WIDTH ADC_WIDTH_BIT_12

/* 5 seconds to 'warm up' the camera */
#define CAM_WARMUP_MS 6000

/* Main clock divider for Camera */
/* Camera clock = (XCLK / (CLKRC+1)) */
/* Making this lower makes the framerate and internal clock higher */
#define CAM_CLKRC 1

/* PCLK generator - PCLK = (XCLK / (CLKRC+1)) * 48 / CAM_PCLK_DIV */
/* Making this lower makes the PCLK higher */
/* Too high and we get corruption trying to consume data that quickly */
/* Too low and we can't get the whole frame out before the next one starts... I think */
#define CAM_PCLK_DIV 4

/* Max number of attempts to init the camera */
#define CAM_MAX_TRIES 10

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22


static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,

    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 5, //0-63 lower number means higher quality
    .fb_count = 1       //if more than one, i2s runs in continuous mode. Use only with JPEG
};


/* If we wake with more than this voltage, do a wifi publish. */
#define WIFI_THRESHOLD_MV 4400

/* If not, we measure whether we lost or gained charge since
   last sleep. This measurement includes the cost to start up */
RTC_DATA_ATTR uint16_t voltage_at_last_sleep;

/* We only call it gaining voltage if we have more than this much more */
#define VOLTAGE_INCREASE_MARGIN_MV 50

/* If we gain voltage, we sleep for the same time as last time
 * If we lose voltage, we sleep for the same time as last time * 2.
 * As we keep waking to find we lost voltage, we keep doubling the sleep time
 * until it starts going up again.
 * Once we hit the wifi threshold, we start sleeping for the
 * minimum sleep time again. */
RTC_DATA_ATTR uint32_t sleep_sec;
#define MIN_SLEEP_SEC 60
#define MAX_SLEEP_SEC 32*60

/* If we find ourselves at max sleep time with power still dropping,
 * try for an upload. Don't waste power on the camera.*/
bool yolo_mode = false;

/* Historical records */
#define HISTORY_SIZE 64  /* 2kB history */


/* 8 bytes */
struct data_record_t
{
    uint32_t time;
    uint16_t voltage;
    float temperature;
    float humidity;
};

RTC_DATA_ATTR struct data_record_t history[HISTORY_SIZE];
RTC_DATA_ATTR uint16_t history_ptr = 0;




static void camera_off(void)
{
    ESP_LOGI(TAG, "Camera off");
    gpio_set_level(CAM_PIN_PWDN, 1);
}


/* Use GPIO 13 - ADC2_CH4 */
static uint32_t read_batt_voltage(void)
{
    adc2_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    static esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type =
        esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN,
                                 ADC_WIDTH, 0, &adc_chars);

    assert(val_type == ESP_ADC_CAL_VAL_EFUSE_VREF);

    int reading;
    ESP_ERROR_CHECK(adc2_get_raw(ADC_CHANNEL, ADC_WIDTH, &reading));
    uint32_t mv = esp_adc_cal_raw_to_voltage(reading, &adc_chars);

    /* 1/2 Divider */
    return mv * 2;
}

static void log_data(void)
{
    struct dht_reading dht;

    int good_reads = 0;
    for(int i = 0; i < 5 && good_reads < DHT_NEEDED_READS; i++)
    {
        dht = DHT_read();

        if(dht.temperature != 255 && dht.temperature != 0 && dht.status == DHT_OK)
        {
            good_reads++;
            ESP_LOGI(TAG, "Got good DHT reading %d/%d", good_reads, DHT_NEEDED_READS);
        }
        else
        {
            ESP_LOGW(TAG, "DHT read failed; trying again");
        }

        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    if(good_reads < DHT_NEEDED_READS)
    {
        ESP_LOGW(TAG, "Giving up on DHT");
        return;
    }

    if(history_ptr == HISTORY_SIZE)
    {
        ESP_LOGE(TAG, "Out of history space! Not recording");
        return;
    }
    struct data_record_t *h = &history[history_ptr++];


    h->time = (uint32_t)time(NULL);
    h->voltage = (uint16_t)read_batt_voltage();
    h->temperature = dht.temperature;
    h->humidity = dht.humidity;

    ESP_LOGI(TAG, "Logging: time = %d voltage = %d, temperature = %f, humidity = %f",
             (int)h->time,
             (int)h->voltage,
             h->temperature,
             h->humidity);
}



/* Deep sleep for some number of seconds */
static void go_to_sleep(uint32_t sleep_secs)
{
    ESP_LOGI(TAG, "Sleeping for %d seconds", sleep_secs);

    /* Clear any wakeup timers which might be on */
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_sleep_enable_timer_wakeup(sleep_secs * 1000000);

    adc_power_off();
    esp_wifi_stop();
    //esp_bt_controller_disable();

    /* Hold camera off during deep sleep */
    //rtc_gpio_init(CAM_PIN_PWDN);
    //rtc_gpio_set_direction(CAM_PIN_PWDN, RTC_GPIO_MODE_OUTPUT_ONLY);
    //rtc_gpio_set_direction_in_sleep(CAM_PIN_PWDN, RTC_GPIO_MODE_OUTPUT_ONLY);
    //rtc_gpio_set_level(CAM_PIN_PWDN, 1);
    //rtc_gpio_isolate(CAM_PIN_PWDN);

    /* PSRAM off */
    //rtc_gpio_init(16);
    //rtc_gpio_set_direction(16, RTC_GPIO_MODE_OUTPUT_ONLY);
    //rtc_gpio_set_level(16, 1);


    esp_deep_sleep_start();
}


static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static int output_len;       // Stores number of bytes read

    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            output_len = 0;
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            // If user_data buffer is configured, copy the response into the buffer
            if (evt->user_data) {
                memcpy(evt->user_data + output_len, evt->data, evt->data_len);
            }
            output_len += evt->data_len;

            break;
    }
    return ESP_OK;
}

#define MAX_HTTP_OUTPUT_BUFFER 2048

#define PSTATUS(attr) ESP_LOGI(TAG, "sensor." #attr " = %d", s->status.attr)
#define PREG(bank, reg) { s->set_reg(s, 0xff, 0xff, bank); ESP_LOGI(TAG, "sensor[%02x:%02x] = %02x",bank,reg,s->get_reg(s, reg, 0xff)); }
static void dump_sensor_status(void)
{
    sensor_t *s = esp_camera_sensor_get();
    PSTATUS(framesize);
    PSTATUS(scale);
    PSTATUS(binning);
    PSTATUS(quality);
    PSTATUS(brightness);
    PSTATUS(contrast);
    PSTATUS(saturation);
    PSTATUS(sharpness);
    PSTATUS(denoise);
    PSTATUS(special_effect);
    PSTATUS(wb_mode);
    PSTATUS(awb);
    PSTATUS(awb_gain);
    PSTATUS(aec);
    PSTATUS(aec2);
    PSTATUS(ae_level);
    PSTATUS(aec_value);
    PSTATUS(agc);
    PSTATUS(agc_gain);
    PSTATUS(gainceiling);
    PSTATUS(bpc);
    PSTATUS(wpc);
    PSTATUS(raw_gma);
    PSTATUS(lenc);
    PSTATUS(hmirror);
    PSTATUS(vflip);
    PSTATUS(dcw);
    PSTATUS(colorbar);
    PREG(0x01, 0x11);
    PREG(0x00, 0x86);
    PREG(0x00, 0xd3);
    PREG(0x00, 0x42);
    PREG(0x00, 0x44);

}

static void upload_image(camera_fb_t *fb)
{
    ESP_LOGI(TAG, "Uploading image to %s:%d%s(%zu kB)...", HTTP_HOST, HTTP_PORT, HTTP_IMAGE_PATH, fb->len / 1024);
    static char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

    char q_buff[100];
    snprintf(q_buff, 100, "t=%f&h=%f&v=%d&l=%zu",
             history[history_ptr-1].temperature,
             history[history_ptr-1].humidity,
             history[history_ptr-1].voltage,
             fb->len);

    q_buff[99] = '\0';

    esp_http_client_config_t config = {
        .host = HTTP_HOST,
        .port = HTTP_PORT,
        .path = HTTP_IMAGE_PATH,
        .method = HTTP_METHOD_POST,
        .query = q_buff,
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer,        // Pass address of local buffer to get response
        .timeout_ms = HTTP_TIMEOUT_SEC * 1000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // POST
    esp_http_client_set_header(client, "Content-Type", "image/jpeg");
    esp_http_client_set_post_field(client, (const char*)fb->buf, fb->len);
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

static void upload_logged_data(void)
{
    size_t data_len = history_ptr * sizeof(history[0]);

    ESP_LOGI(TAG, "Uploading logged data to %s:%d%s(%zu kB)...",
             HTTP_HOST, HTTP_PORT, HTTP_DATA_PATH, data_len / 1024);

    static char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

    char q_buff[100];
    snprintf(q_buff, 100, "t=%d", (uint32_t)time(NULL));
    q_buff[99] = '\0';

    esp_http_client_config_t config = {
        .host = HTTP_HOST,
        .port = HTTP_PORT,
        .path = HTTP_DATA_PATH,
        .method = HTTP_METHOD_POST,
        .query = q_buff,
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer,        // Pass address of local buffer to get response
        .timeout_ms = HTTP_TIMEOUT_SEC * 1000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // POST
    esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
    esp_http_client_set_post_field(client, (const char*)history, data_len);
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));

        /* Reset the history pointer if we succeeded */
        history_ptr = 0;
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

static void snap_task(void* arg)
{
    camera_fb_t *fb = NULL;
    esp_err_t e;
    bool camera_up = false;


    if(!yolo_mode) // Don't try to do camera stuff in yolo mode
    {
        /* Don't light sleep - it breaks the camera */
        esp_pm_lock_acquire(pm_lock);

        for(int i = 0; i < CAM_MAX_TRIES && !camera_up; i++)
        {
            ESP_LOGI(TAG, "Init camera...");
            if((e = esp_camera_init(&camera_config)) != ESP_OK)
            {
                ESP_LOGE(TAG, "Error %d initializing camera", e);
                /* Retry */
            }
            else
            {
                ESP_LOGI(TAG, "Camera initialized");
                /* Camera now initialized */
                camera_up = true;
            }
        }
    }

    if(camera_up)
    {
        /* Camera is initialized, try to get a frame */

        sensor_t *s = esp_camera_sensor_get();
        s->set_reg(s,0xff,0xff,0x01);//banksel
        s->set_reg(s,0x11,0xff, CAM_CLKRC); //frame rate
        s->set_reg(s,0xff,0xff,0x00);//banksel
        s->set_reg(s,0xd3,0xff,CAM_PCLK_DIV); // DVP clock divider - too low and PCLK goes too high and stops working
        dump_sensor_status();

        ESP_LOGI(TAG, "Camera 'warmup'...");
        vTaskDelay(CAM_WARMUP_MS/portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Camera ready");

        for(int i = 0; i < CAM_MAX_TRIES && !fb; i++)
        {
            /* Take a picture */
            ESP_LOGI(TAG, "Capturing image...");
            fb = esp_camera_fb_get();

            if(!fb)
            {
                ESP_LOGE(TAG, "Unable to get frame");
            }
        }

    }
    else
    {
        /* Turn this off now */
        camera_off();
    }

    if(!yolo_mode) // We don't lock light sleep in yolo mode
    {
        /* Done now - can go to sleep all we want */
        esp_pm_lock_release(pm_lock);
    }

    log_data();

    /* Connect to wifi */
    ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));

    if(fb)
    {
        upload_image(fb);

        esp_camera_fb_return(fb);

        ESP_ERROR_CHECK(esp_camera_deinit());

        camera_off();
    }

    /* Upload data that we logged */
    upload_logged_data();

    ESP_ERROR_CHECK(example_disconnect());

    log_data();

    /* Phew - done! */

    /* Zzzz */
    voltage_at_last_sleep = history[history_ptr-1].voltage;
    go_to_sleep(sleep_sec);
}

static void setup_gpio(void)
{
    gpio_config_t io_conf;

    /* Camera power */
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1LL << CAM_PIN_PWDN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    /* DHT */
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1 << DHT_GPIO;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    /* ADC */
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1 << ADC_GPIO;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //gpio_deep_sleep_hold_en();

}



void app_main(void)
{
    setup_gpio();

    camera_off();

    //DHT_init(DHT_GPIO, MODE_DHT22);
    DHT22_init(DHT_GPIO);

    ESP_ERROR_CHECK(nvs_flash_init());

    if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED)
    {
        /* We just powered on - not deep sleep
         * Reset time & history pointer */
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        history_ptr = 0;
        voltage_at_last_sleep = 0;
        sleep_sec = MIN_SLEEP_SEC;
    }

    log_data();

    uint16_t voltage_at_startup = history[history_ptr-1].voltage;


    ESP_LOGI(TAG, "Battery voltage: %dmV; Wifi Threshold: %dmV", voltage_at_startup, WIFI_THRESHOLD_MV);

    if(voltage_at_startup < WIFI_THRESHOLD_MV)
    {
        int16_t voltage_change = (int16_t)voltage_at_startup - (int16_t)voltage_at_last_sleep;

        ESP_LOGI(TAG,"Voltage change: %dmV", voltage_change);

        if(voltage_change > VOLTAGE_INCREASE_MARGIN_MV)
        {
            /* Gained enough voltage - we're good with the current sleep time */
            voltage_at_last_sleep = voltage_at_startup;
            go_to_sleep(sleep_sec);
        }
        else
        {
            if(sleep_sec < MAX_SLEEP_SEC)
            {
                sleep_sec = sleep_sec * 2;

                /* Sleep twice as long this time */
                ESP_LOGI(TAG, "Lost voltage");

                go_to_sleep(sleep_sec);
            }
            else
            {
                /* We lost voltage and there's nothing we can do about it
                 * Try for an upload.
                 */
                ESP_LOGI(TAG, "Lost voltage even though we maxed sleep time. YOLO MODE");
                yolo_mode = true;
            }
        }
    }
    else
    {
        /* We're doing full wifi - sleep for a short time after this */
        sleep_sec = MIN_SLEEP_SEC;
    }

    /* If we find ourselves still losing voltage after maxing out sleep time,
     * Do a publish - we're probably done for the day */

    /* GO GO GO!!! */

    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 240,
        .light_sleep_enable = true
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "no light sleep", &pm_lock);

    esp_log_level_set("camera",ESP_LOG_VERBOSE);

    xTaskCreatePinnedToCore(&snap_task, "snap_task", 8192, NULL, 0, NULL, 0);
}
