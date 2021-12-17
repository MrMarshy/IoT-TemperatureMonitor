#include "ESP32_GPIO.h"

void ESP32_GPIO_init_output(gpio_num_t gpio_num){
    gpio_config_t io_conf;

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << gpio_num);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);
}

void ESP32_GPIO_init_input(gpio_num_t gpio_num, ESP32_GPIO_PullType_t pull_type){

    gpio_config_t io_conf;
    
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << gpio_num);
    switch(pull_type){
        case PULL_DOWN:
            io_conf.pull_up_en = 0;
            io_conf.pull_down_en = 1;
            break;
        case PULL_UP:
            io_conf.pull_down_en = 0;
            io_conf.pull_up_en = 1;
            break;
        case NONE:
            io_conf.pull_down_en = 0;
            io_conf.pull_up_en = 0;
            break;
        default:
            io_conf.pull_down_en = 0;
            io_conf.pull_up_en = 0;
    }
    gpio_config(&io_conf);

}

void ESP32_GPIO_init_input_with_interrupt(gpio_num_t gpio_num, ESP32_GPIO_PullType_t pull_type, gpio_int_type_t intr_type, gpio_isr_t isr_func){

    gpio_config_t io_conf;
    
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << gpio_num);
    switch(pull_type){
        case PULL_DOWN:
            io_conf.pull_up_en = 0;
            io_conf.pull_down_en = 1;
            break;
        case PULL_UP:
            io_conf.pull_down_en = 0;
            io_conf.pull_up_en = 1;
            break;
        case NONE:
            io_conf.pull_down_en = 0;
            io_conf.pull_up_en = 0;
            break;
        default:
            io_conf.pull_down_en = 0;
            io_conf.pull_up_en = 0;
    }

    io_conf.intr_type = intr_type;

    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(gpio_num, isr_func, NULL);
}