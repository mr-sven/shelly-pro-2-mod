/**
 * @file shift_reg.h
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "driver/gpio.h"

// SN74HC595 pin config
#define SHIFT_REG_DATA          GPIO_NUM_13 // SER
#define SHIFT_REG_CLOCK         GPIO_NUM_14 // SRCLK
#define SHIFT_REG_LATCH         GPIO_NUM_4  // RCLK
#define SHIFT_REG_PIN_SEL       ((1ULL<<SHIFT_REG_DATA) | (1ULL<<SHIFT_REG_CLOCK) | (1ULL<<SHIFT_REG_LATCH))

#define SHIFT_REG_LED_MASK      0b00011100
#define SHIFT_REG_RELAY_MASK    0b00000011
#define SHIFT_REG_LED_RED       0b00010000
#define SHIFT_REG_LED_GREEN     0b00001000
#define SHIFT_REG_LED_BLUE      0b00000100
#define SHIFT_REG_RELAY_0       0b00000001
#define SHIFT_REG_RELAY_1       0b00000010

void shift_reg_init(void);
void shift_reg_set_led(uint8_t led);
void shift_reg_set_relay(uint8_t relay, bool on);