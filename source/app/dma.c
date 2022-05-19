/*_____________________________________________________________________________
 │                                                                            |
 │ COPYRIGHT (C) 2022 Mihai Baneu                                             |
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
 |  Author: Mihai Baneu                           Last modified: 19.May.2022  |
 |                                                                            |
 |___________________________________________________________________________*/

#include "stm32f4xx.h"
#include "stm32rtos.h"
#include "queue.h"
#include "dma.h"
#include "i2c.h"

/* Queue used to communicate dma messages. */
QueueHandle_t dma_request_queue;
QueueHandle_t dma_response_queue;

void dma_init()
{
    /* create the dma request/response queues */
    dma_request_queue = xQueueCreate(1, sizeof(dma_request_event_t));
    dma_response_queue = xQueueCreate(1, sizeof(dma_response_event_t));

//    /* make sure the DMA stream is disabled */
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_EN_Msk, 0);
//    do {
//    } while ((DMA2_Stream0->CR & DMA_SxCR_EN_Msk) != 0);
//
//    /* clear the interupt register */
//    SET_BIT(DMA2->LIFCR, DMA_LIFCR_CFEIF0_Msk | DMA_LIFCR_CDMEIF0_Msk | DMA_LIFCR_CTEIF0_Msk | DMA_LIFCR_CHTIF0_Msk | DMA_LIFCR_CTCIF0_Msk);
//
//    /* configure the pointers/data amount for DMA */
//    DMA2_Stream0->PAR  = (uint32_t)&(ADC1->DR);
//    DMA2_Stream0->M0AR = (uint32_t)dma_buffer0;
//    DMA2_Stream0->M1AR = (uint32_t)dma_buffer1;
//    DMA2_Stream0->NDTR = ADC_SAMPLES_COUNT;
//
//    /* select the channel 0 for the stram 0 - ADC1*/
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_CHSEL_Msk, 0);
//
//    /* double buffer mode: start with memmory pointer 0 (periferal to memory)*/
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_DBM_Msk, DMA_SxCR_DBM);
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_CT_Msk,  0);
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_DIR,     0);
//
//    /* configure periferal */
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_PSIZE_Msk, DMA_SxCR_PSIZE_0);   // 16 bit
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_PINC_Msk,  0);                  // no increment
//    
//    /* configure memory */
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_MSIZE_Msk, DMA_SxCR_MSIZE_0);   // 16 bit
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_MINC_Msk,  DMA_SxCR_MINC);      // increment
//
//    /* enable interupt */
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_TCIE_Msk, DMA_SxCR_TCIE);
}

void dma_enable()
{
//    memset(dma_buffer0, 0, ADC_SAMPLES_COUNT);
//    memset(dma_buffer1, 0, ADC_SAMPLES_COUNT);
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_EN_Msk, DMA_SxCR_EN);
}

void dma_disable()
{
//    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_EN_Msk, 0);
}

void dma_isr_handler()
{
//    if (DMA2->LISR & DMA_LISR_TCIF0_Msk) {
//        dma_event_t dma_event;
//
//        dma_event.length = ADC_SAMPLES_COUNT;
//        if (DMA2_Stream0->CR & DMA_SxCR_CT_Msk) {
//            dma_event.buffer = dma_buffer0;
//        } else {
//            dma_event.buffer = dma_buffer1;
//        }
//
//        /* clear the interupt register */
//        SET_BIT(DMA2->LIFCR, DMA_LIFCR_CFEIF0_Msk | DMA_LIFCR_CDMEIF0_Msk | DMA_LIFCR_CTEIF0_Msk | DMA_LIFCR_CHTIF0_Msk | DMA_LIFCR_CTCIF0_Msk);
//        xQueueSendFromISR(dma_queue, &dma_event, (TickType_t) 0);
//    }
}

void dma_run(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        dma_request_event_t req_event;
        if (xQueueReceive(dma_request_queue, &req_event, portMAX_DELAY) == pdPASS) {
            dma_response_event_t res_event;

            switch (req_event.type) {
                case dma_request_type_i2c_write:
                    res_event.length = i2c_write(req_event.address, (uint8_t *)req_event.buffer, req_event.length);
                    res_event.status = (res_event.length == req_event.length) ? dma_request_status_success : dma_request_status_error;
                    break;
                case dma_request_type_i2c_read:
                    res_event.length = i2c_read(req_event.address, (uint8_t *)res_event.buffer, req_event.length);
                    res_event.status = (res_event.length == req_event.length) ? dma_request_status_success : dma_request_status_error;
                    break;    

            }

            xQueueSendToBack(dma_response_queue, &res_event, (TickType_t) 0);
        }
    }
}
