/**
 * @file shift_reg.c
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "rom/ets_sys.h"

#include "shift_reg.h"

static QueueHandle_t queue;
static uint8_t current_value;

static void shift_reg_send(uint8_t data)
{
    current_value = data;
    for (int shift = 0; shift < 8; shift++)
    {
        gpio_set_level(SHIFT_REG_DATA, data & 0x80);
        data <<= 1;
        gpio_set_level(SHIFT_REG_CLOCK, true);
        ets_delay_us(1);
        gpio_set_level(SHIFT_REG_CLOCK, false);
        ets_delay_us(1);
    }

    gpio_set_level(SHIFT_REG_LATCH, true);
    ets_delay_us(1);
    gpio_set_level(SHIFT_REG_LATCH, false);
    ets_delay_us(1);
}

static void shift_reg_task(void *parameter)
{
    uint8_t new_value = 0;
    for(;;)
    {
        new_value = current_value;
        xQueueReceive(queue, &new_value, portMAX_DELAY);
        if (new_value != current_value)
        {
            shift_reg_send(new_value);
        }
    }
}

void shift_reg_init(void)
{
    queue = xQueueCreate(10, sizeof(uint8_t));

    gpio_config_t io_conf = {
		.intr_type    = GPIO_INTR_DISABLE,
		.mode         = GPIO_MODE_OUTPUT,
		.pin_bit_mask = SHIFT_REG_PIN_SEL,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.pull_up_en   = GPIO_PULLUP_DISABLE};
    gpio_config(&io_conf);

    shift_reg_send(0);

    xTaskCreate(shift_reg_task, "shift_reg_task", 1048, NULL, 5, NULL);
}

void shift_reg_set_led(uint8_t led)
{
    // invert bits because output is inverted
    led = ~led;

    // mask bits, so only leds gets update
    led &= SHIFT_REG_LED_MASK;

    // get relay bits and merge led bits
    led = (current_value & SHIFT_REG_RELAY_MASK) | led;

    // send value to queue
    xQueueSend(queue, &led, 0);
}

void shift_reg_set_relay(uint8_t relay, bool on)
{
    // mask bits, so only relay gets update
    relay &= SHIFT_REG_RELAY_MASK;
    if(on)
    {
        relay = (current_value & ~relay) | relay;
    }
    else
    {
        relay = current_value & ~relay;
    }

    // send value to queue
    xQueueSend(queue, &relay, 0);
}