#include "esp_log.h"
#include "hal/gpio_hal.h"

/* Function used to tell the linker to include this file
 * with all its symbols.
 */
void bootloader_hooks_include(void){
}


void bootloader_before_init(void) {
    /* Keep in my mind that a lot of functions cannot be called from here
     * as system initialization has not been performed yet, including
     * BSS, SPI flash, or memory protection. */
    gpio_ll_output_enable (&GPIO, GPIO_NUM_45);
    gpio_ll_set_level (&GPIO, GPIO_NUM_45, 0);	 
    gpio_ll_output_enable (&GPIO, GPIO_NUM_48);
    gpio_ll_set_level (&GPIO, GPIO_NUM_48, 0);	 
    gpio_ll_output_enable (&GPIO, GPIO_NUM_47);
    gpio_ll_set_level (&GPIO, GPIO_NUM_47, 0);	 	
    ESP_LOGI("HOOK", "Set GPIO_48, 47, 45 Low");
}

void bootloader_after_init(void) {
    // ESP_LOGI("HOOK", "This hook is called AFTER bootloader initialization");
}
