/**
 * @file main.c
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-06-10
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "driver/gpio.h"

#include "nvs_flash.h"

#include "onewire_bus.h"
#include "ds18b20.h"

#include "wifi.h"
#include "mqtt.h"
#include "ota.h"
#include "shift_reg.h"

#define KELVIN_TO_CELSIUS           273.15
#define KELVIN_AT_25_DEGREE_CELSIUS (KELVIN_TO_CELSIUS + 25.0) // Temperature in Kelvin for 25 degree Celsius
#define RELAY_TEMP_NTC_DEVIDER      10000.0 // 10kOhm, voltage divider resistor value
#define RELAY_TEMP_NTC_BETA         3350.0
#define RELAY_TEMP_NTC_R            10000.0 // 10kOhm, Resistance of Thermistor at 25 degree Celsius
#define RELAY_TEMP_ADC_MVOLTAGE     3300.0
#define RELAY_TEMP_ADC_UNIT         ADC_UNIT_1
#define RELAY_TEMP_ADC_WIDTH        ADC_ATTEN_DB_12
#define RELAY_1_TEMP_ADC_CHANNEL    ADC_CHANNEL_0 // GPIO36
#define RELAY_2_TEMP_ADC_CHANNEL    ADC_CHANNEL_1 // GPIO37

#define IN_SW_RESET                 GPIO_NUM_35
//#define IN_SW_1                     GPIO_NUM_38
//#define IN_SW_2                     GPIO_NUM_39
#define IN_SW_1                     GPIO_NUM_32
#define IN_SW_2                     GPIO_NUM_33

#define ONEWIRE_MAX_DS18B20         5

static const char MQTT_STATUS_OFF[] = "off";
static const char MQTT_STATUS_ON[] =  "on";
static const char MQTT_SUB_OUT[] =  "/output";
static const char MQTT_SUB_VAL_TEMP[] =  "/temp";
static const char MQTT_SUB_VAL_STATUS[] =  "/status";
static const char MQTT_SUB_VAL_POWER[] =  "/power";

static const char TAG[] = "MAIN";

#define MQTT_TOPIC_PREFIX_LEN   20
#define MQTT_TOPIC_PREFIX_SIZE  (MQTT_TOPIC_PREFIX_LEN + 1)

#define MQTT_TOPIC_STATUS_LEN   27
#define MQTT_TOPIC_STATUS_SIZE  (MQTT_TOPIC_STATUS_LEN + 1)

#define MQTT_TOPIC_BUF_LEN      255

// Prefix used for mqtt topics
static char mqtt_topic_prefix[MQTT_TOPIC_PREFIX_SIZE]; // strlen("device/shelly/aabbcc") = 20 + 1

// status topic
static char mqtt_topic_status[MQTT_TOPIC_STATUS_SIZE]; // strlen("device/shelly/aabbcc/status") = 27 + 1

static void temp_read_task(void *parameter);

void mqtt_publish_sub_value_str(const char *sub, int id, const char *value_name, const char *value, int retain);
void mqtt_publish_sub_value_double(const char *sub, int id, const char *value_name, const char *format, const double value, int retain);
void mqtt_subscribe_sub_value_topic(const char *sub, int id, const char *value_name);

static void handle_output_topic(uint8_t output, char *topic, char *data);

bool starts_with(const char *pre, const char *str)
{
    size_t lenpre = strlen(pre);
    size_t lenstr = strlen(str);
    return lenstr < lenpre ? false : memcmp(pre, str, lenpre) == 0;
}

/**
 * @brief MQTT Message data handler callback
 *
 * @param topic
 * @param data
 */
void mqtt_data(char *topic, char *data)
{
    // match device
    if (strncmp(topic, mqtt_topic_prefix, MQTT_TOPIC_PREFIX_LEN) != 0)
    {
        return;
    }

    // move pointer to remove prefix
    topic += MQTT_TOPIC_PREFIX_LEN;
    ESP_LOGI(TAG, "subtopic: %s", topic);

    if (starts_with(MQTT_SUB_OUT, topic))
    {
        topic += strlen(MQTT_SUB_OUT) + 1; // plus 1 for trailing slash
        uint8_t output = strtoul(topic, &topic, 10);
        if (output <= 1)
        {
            handle_output_topic(output, topic, data);
        }
    }
}

/**
 * @brief Callback from MQTT Handler on connect
 *
 */
void mqtt_connected(void)
{
    mqtt_publish(mqtt_topic_status, MQTT_STATUS_ON, strlen(MQTT_STATUS_ON), 0, 1);
    shift_reg_set_led(SHIFT_REG_LED_GREEN);
    mqtt_publish_sub_value_str(MQTT_SUB_OUT, 0, MQTT_SUB_VAL_STATUS, MQTT_STATUS_OFF, 1);
    mqtt_publish_sub_value_str(MQTT_SUB_OUT, 1, MQTT_SUB_VAL_STATUS, MQTT_STATUS_OFF, 1);

    mqtt_subscribe_sub_value_topic(MQTT_SUB_OUT, 0, MQTT_SUB_VAL_POWER);
    mqtt_subscribe_sub_value_topic(MQTT_SUB_OUT, 1, MQTT_SUB_VAL_POWER);

    xTaskCreate(temp_read_task, "temp_read_task", 4096, NULL, 5, NULL);
}

char * build_sub_value_topic(const char *sub, int id, const char *value_name)
{
    char * topic = calloc(MQTT_TOPIC_BUF_LEN, sizeof(char));
    snprintf(topic, MQTT_TOPIC_BUF_LEN, "%s%s/%d%s", mqtt_topic_prefix, sub, id, value_name);
    return topic;
}

void mqtt_subscribe_sub_value_topic(const char *sub, int id, const char *value_name)
{
    char * topic = build_sub_value_topic(sub, id, value_name);
    mqtt_subscribe(topic, 0);
    free(topic);
}

void mqtt_publish_sub_value_str(const char *sub, int id, const char *value_name, const char *value, int retain)
{
    char * topic = build_sub_value_topic(sub, id, value_name);
    mqtt_publish(topic, value, strlen(value), 0, retain);
    free(topic);
}

void mqtt_publish_sub_value_double(const char *sub, int id, const char *value_name, const char *format, const double value, int retain)
{
    char str_value[20];
    snprintf(str_value, 20, format, value);
    mqtt_publish_sub_value_str(sub, id, value_name, str_value, retain);
}

void mqtt_publish_ds18b20_value(const uint64_t addr, const float value, int retain)
{
    char topic[MQTT_TOPIC_BUF_LEN];
    snprintf(topic, MQTT_TOPIC_BUF_LEN, "device/ds18b20/%016llX", addr);

    char str_value[20];
    snprintf(str_value, 20, "%.2f", value);
    mqtt_publish(topic, str_value, strlen(str_value), 0, retain);
}

static void handle_output_topic(uint8_t output, char *topic, char *data)
{
    ESP_LOGI(TAG, "output: %d subtopic: %s", output, topic);
    if (strcmp(topic, MQTT_SUB_VAL_POWER) == 0)
    {
        uint8_t relay = SHIFT_REG_RELAY_0;
        if (output == 1)
        {
            relay = SHIFT_REG_RELAY_1;
        }

        if (strcmp(data, MQTT_STATUS_ON) == 0)
        {
            shift_reg_set_relay(relay, true);
            mqtt_publish_sub_value_str(MQTT_SUB_OUT, output, MQTT_SUB_VAL_STATUS, MQTT_STATUS_ON, 1);
        }
        else if (strcmp(data, MQTT_STATUS_OFF) == 0)
        {
            shift_reg_set_relay(relay, false);
            mqtt_publish_sub_value_str(MQTT_SUB_OUT, output, MQTT_SUB_VAL_STATUS, MQTT_STATUS_OFF, 1);
        }
    }
}

void app_main(void)
{
    // init OTA data
    ota_precheck();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    shift_reg_init();

    // start WIFI
    ESP_ERROR_CHECK(wifi_init_sta());

    // execute OTA Update
    ota_execute();

    // load MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    // prepare device id
    snprintf(mqtt_topic_prefix, MQTT_TOPIC_PREFIX_SIZE, "device/shelly/%02x%02x%02x", mac[3], mac[4], mac[5]);
    snprintf(mqtt_topic_status, MQTT_TOPIC_STATUS_SIZE, "%s/status", mqtt_topic_prefix);

    // start MQTT
    mqtt_app_start(mqtt_topic_status, MQTT_STATUS_OFF, strlen(MQTT_STATUS_OFF), 0, 1);
}

static inline double adc_mv_to_celsius(double mv)
{
    double rt = RELAY_TEMP_NTC_DEVIDER * mv / (RELAY_TEMP_ADC_MVOLTAGE - mv);
    double tk = 1/(1/KELVIN_AT_25_DEGREE_CELSIUS + log(rt/RELAY_TEMP_NTC_R)/RELAY_TEMP_NTC_BETA); // Temperature in Kelvin
    return tk - KELVIN_TO_CELSIUS; // Celsius
}

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_relay_1_handle = NULL;
static adc_cali_handle_t cali_relay_2_handle = NULL;

static void init_relay_adc(void)
{
    //-------------ADC Init---------------//
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = RELAY_TEMP_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    //-------------ADC Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = RELAY_TEMP_ADC_WIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, RELAY_1_TEMP_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, RELAY_2_TEMP_ADC_CHANNEL, &config));

    //-------------ADC Calibration Init---------------//
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = RELAY_TEMP_ADC_UNIT,
        .atten = RELAY_TEMP_ADC_WIDTH,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_relay_1_handle));
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_relay_2_handle));
}

static onewire_bus_handle_t bus = NULL;
static int ds18b20_device_num = 0;
static ds18b20_device_handle_t ds18b20s[ONEWIRE_MAX_DS18B20];

static void init_onewire(void)
{
    // install 1-wire bus
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = IN_SW_1,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // create 1-wire device iterator, which is used for device search
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI(TAG, "Device iterator created, start searching...");
    do
    {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK)
        {
            // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK)
            {
                ESP_LOGI(TAG, "Found a DS18B20[%d], address: %016llX", ds18b20_device_num, next_onewire_device.address);
                ds18b20_device_num++;
            }
            else
            {

                ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND && ds18b20_device_num < ONEWIRE_MAX_DS18B20);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", ds18b20_device_num);
}

static void temp_read_task(void *parameter)
{
    init_relay_adc();

    init_onewire();

    int relay_1_mv = 0;
    int relay_2_mv = 0;

    for(;;)
    {
        TickType_t ds18b20_read_time;
        if (ds18b20_device_num > 0)
        {
            ds18b20_trigger_all_temperature_conversion(bus);
            ds18b20_read_time = xTaskGetTickCount();
        }

        adc_oneshot_get_calibrated_result(adc_handle, cali_relay_1_handle, RELAY_1_TEMP_ADC_CHANNEL, &relay_1_mv);
        adc_oneshot_get_calibrated_result(adc_handle, cali_relay_2_handle, RELAY_1_TEMP_ADC_CHANNEL, &relay_2_mv);

        double relay_1_temp = adc_mv_to_celsius((double)relay_1_mv);
        double relay_2_temp = adc_mv_to_celsius((double)relay_2_mv);

        mqtt_publish_sub_value_double(MQTT_SUB_OUT, 0, MQTT_SUB_VAL_TEMP, "%.2f", relay_1_temp, 0);
        mqtt_publish_sub_value_double(MQTT_SUB_OUT, 1, MQTT_SUB_VAL_TEMP, "%.2f", relay_2_temp, 0);

        if (ds18b20_device_num > 0)
        {
            vTaskDelayUntil(&ds18b20_read_time, pdMS_TO_TICKS(800));
            for (int i = 0; i < ds18b20_device_num; i ++)
            {
                float temperature;
                uint64_t addr;
                ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20s[i], &temperature));
                ESP_ERROR_CHECK(ds18b20_get_address(ds18b20s[i], &addr));
                mqtt_publish_ds18b20_value(addr, temperature, 0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
