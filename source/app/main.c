/*_____________________________________________________________________________
 │                                                                            |
 │ COPYRIGHT (C) 2021 Mihai Baneu                                             |
 │                                                                            |
 | Permission is hereby  granted,  free of charge,  to any person obtaining a |
 | copy of this software and associated documentation files (the "Software"), |
 | to deal in the Software without restriction,  including without limitation |
 | the rights to  use, copy, modify, merge, publish, distribute,  sublicense, |
 | and/or sell copies  of  the Software, and to permit  persons to  whom  the |
 | Software is furnished to do so, subject to the following conditions:       |
 |                                                                            |
 | The above  copyright notice  and this permission notice  shall be included |
 | in all copies or substantial portions of the Software.                     |
 |                                                                            |
 | THE SOFTWARE IS PROVIDED  "AS IS",  WITHOUT WARRANTY OF ANY KIND,  EXPRESS |
 | OR   IMPLIED,   INCLUDING   BUT   NOT   LIMITED   TO   THE  WARRANTIES  OF |
 | MERCHANTABILITY,  FITNESS FOR  A  PARTICULAR  PURPOSE AND NONINFRINGEMENT. |
 | IN NO  EVENT SHALL  THE AUTHORS  OR  COPYRIGHT  HOLDERS  BE LIABLE FOR ANY |
 | CLAIM, DAMAGES OR OTHER LIABILITY,  WHETHER IN AN ACTION OF CONTRACT, TORT |
 | OR OTHERWISE, ARISING FROM,  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR  |
 | THE USE OR OTHER DEALINGS IN THE SOFTWARE.                                 |
 |____________________________________________________________________________|
 |                                                                            |
 |  Author: Mihai Baneu                           Last modified: 14.Nov.2021  |
 |                                                                            |
 |___________________________________________________________________________*/

#include "stm32f4xx.h"
#include "stm32rtos.h"
#include "task.h"
#include "queue.h"
#include "system.h"
#include "gpio.h"
#include "isr.h"
#include "i2c.h"
#include "printf.h"
#include "led.h"
#include "lcd.h"
#include "rencoder.h"

#define EEPROM_SIZE         512
#define EEPROM_PAGE_SIZE    16
#define EEPROM_I2C_ADDRESS  0b01010000

static void query_eeprom(uint16_t counter)
{
    // calculate the read address of the eeprom
    uint8_t address = EEPROM_I2C_ADDRESS + (uint8_t)(counter >> 8);
    uint16_t nread, nwrite;
    lcd_event_t lcd_event = { {0}, {0} };

    // read i2c eeprom
    nwrite = i2c_write(address, (uint8_t *)&counter, 1);
    if (nwrite != 1) {
        sprintf(lcd_event.row1_txt, "%s", "Write error");
        sprintf(lcd_event.row2_txt, "%d", nwrite);
    } else {
        nread = i2c_read(EEPROM_I2C_ADDRESS, (uint8_t *)lcd_event.row1_txt, 16);
        if (nread != 16) {
            sprintf(lcd_event.row1_txt, "%s [%d]", "Read error", nread);
        }
        nread = i2c_read(EEPROM_I2C_ADDRESS, (uint8_t *)lcd_event.row2_txt, 16);
        if (nread != 16) {
            sprintf(lcd_event.row2_txt, "%s [%d]", "Read error", nread);
        }
    }

    xQueueSendToBack(lcd_queue, &lcd_event, (TickType_t) 1);
}

static void user_handler(void *pvParameters)
{
    (void)pvParameters;

    query_eeprom(0);
    for (;;) {
        rencoder_output_event_t event;
        if (xQueueReceive(rencoder_output_queue, &event, portMAX_DELAY) == pdPASS) {
            if (event.type == rencoder_output_rotation) {
                query_eeprom(event.position * 16);
            } else if ((event.type == rencoder_output_key) && (event.key == RENCODER_KEY_RELEASED)) {
                rencoder_reset();
                query_eeprom(0);
            }
        }
    }
}

int main(void)
{
    /* initialize the system */
    system_init();

    /* initialize the gpio */
    gpio_init();

    /* initialize the interupt service routines */
    isr_init();

    /* initialize the i2c interface */
    i2c_init();

    /* init led handler */
    lcd_init();

    /* init lcd display */
    lcd_init();

    /* initialize the encoder */
    rencoder_init(0, (EEPROM_SIZE - 32) / 16);

    /* create the tasks specific to this application. */
    xTaskCreate(led_run,      "led",          configMINIMAL_STACK_SIZE,     NULL, 3, NULL);
    xTaskCreate(lcd_run,      "lcd",          configMINIMAL_STACK_SIZE*2,   NULL, 2, NULL);
    xTaskCreate(user_handler, "user_handler", configMINIMAL_STACK_SIZE*2,   NULL, 2, NULL);
    xTaskCreate(rencoder_run, "rencoder",     configMINIMAL_STACK_SIZE,     NULL, 2, NULL);

    /* start the scheduler. */
    vTaskStartScheduler();

    /* should never get here ... */
    blink(10);
    return 0;
}
