
#include <board.h>
#include "usrdef.h"
#include "app_version.h"
#include "git_version.h"
#include "sys_gpio.h"
#include "sys_dev.h"


#define LED    GPIO_GET_PIN(C, 13)


static int ota_app_vtor_reconfig(void)
{
    SCB->VTOR = (uint32_t)((uint32_t)0x08000000 | ((uint32_t)0x5000 & (uint32_t)0x1FFFFF80));
    return 0;
}
//INIT_BOARD_EXPORT(ota_app_vtor_reconfig);


static void LED_Init(void)
{
	gpio_pin_mode(LED, PIN_MODE_OUTPUT);
	gpio_pin_write(LED, PIN_LOW);
}


int main(void)
{
	uint8_t led_state = 0;
    uint64_t led_timeout = sys_absolute_time();
	LED_Init();

	while(1)
	{
		if(sys_elapsed_time(led_timeout) > 100000)
        {
            led_timeout = sys_absolute_time();

            if(led_state == 0)
            {
                gpio_pin_write(LED, PIN_LOW);
            }
            else
            {
                gpio_pin_write(LED, PIN_HIGH);
            }

            led_state = ~led_state;
        }
		task_delay_ms(50);
	}
}
