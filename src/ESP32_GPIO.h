#include "driver/gpio.h"

#define ESP_INTR_FLAG_DEFAULT   0

typedef enum{
    PULL_UP,
    PULL_DOWN,
    NONE
}ESP32_GPIO_PullType_t;

void ESP32_GPIO_init_output(gpio_num_t gpio_num);
void ESP32_GPIO_init_input(gpio_num_t gpio_num, ESP32_GPIO_PullType_t pull_type);
void ESP32_GPIO_init_input_with_interrupt(gpio_num_t gpio_num, ESP32_GPIO_PullType_t pull_type, gpio_int_type_t intr_type, gpio_isr_t isr_func);