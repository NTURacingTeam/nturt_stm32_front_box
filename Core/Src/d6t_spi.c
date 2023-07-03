#include "d6t_spi.h"

typedef enum {
    START,
    NEXT,
    ABORT
} update_mode;

static TaskHandle_t task_to_notify = NULL;
uint8_t* tx_buffer = NULL;
uint8_t* rx_buffer = NULL;
uint32_t flag_to_raise = 0;
static enum {
    SPI_IDLE = 0,
    SPI_FIRST_BYTE_START,
    SPI_FIRST_BYTE_END,
    SPI_SECOND_BYTE_START,
    SPI_SECOND_BYTE_END,
    SPI_EOT,
    SPI_ERROR
} state = SPI_FIRST_BYTE_START;

int spi4_state_machine_update(const update_mode mode);

//TODO: update error code type
int spi4_state_machine_update(const update_mode mode) {
    //TODO: refactor this thing because this just feels bad. How to link ISR properly
    //note this is the htim17 ISR. htim17 is used for 3us delay here for AMT22
    switch (mode) {
        case NEXT:
            state++;
            break;
        case START: 
            break;
        default:
            break;
    }
    switch(state) {
        case SPI_IDLE :
            HAL_GPIO_WritePin(Encoder_SS_GPIO_Port, Encoder_SS_Pin, GPIO_PIN_RESET);
            HAL_TIM_Base_Start_IT(&htim17);
            break;
        case SPI_FIRST_BYTE_START :
            HAL_SPI_TransmitReceive_IT(&hspi4, tx_buffer, rx_buffer, 1);
            break;
        case SPI_FIRST_BYTE_END :
            HAL_TIM_Base_Start_IT(&htim17);
            break;
        case SPI_SECOND_BYTE_START :
            //low byte transaction
            HAL_SPI_TransmitReceive_IT(&hspi4, &tx_buffer[1], &rx_buffer[1], 1);
            break;
        case SPI_SECOND_BYTE_END :
            HAL_TIM_Base_Start_IT(&htim17);
            break;
        case SPI_EOT :
            HAL_GPIO_WritePin(Encoder_SS_GPIO_Port, Encoder_SS_Pin, GPIO_PIN_SET);
            if(xPortIsInsideInterrupt() == pdTRUE) {
                xTaskNotifyFromISR(task_to_notify, flag_to_raise, eSetBits, NULL);
            }
            else {
                xTaskNotify(task_to_notify, flag_to_raise, eSetBits);
            }
            state = SPI_IDLE;
            break;
        default :
            return 1;
            break;
    }
    return 0;
}

int spi4_state_machine_start(TaskHandle_t task, const uint32_t flag, uint8_t* txBuffer, uint8_t* rxBuffer) {
    if(task_to_notify == NULL || tx_buffer == NULL || rx_buffer == NULL || flag_to_raise == 0 || state != SPI_IDLE) {
        return 1;
    }
    task_to_notify = task;
    flag_to_raise = flag;
    tx_buffer = txBuffer;
    rx_buffer = txBuffer;
    spi4_state_machine_update(START);
    return 0;
}

void ___delay_3us_done(TIM_HandleTypeDef* htim) {
    HAL_TIM_Base_Stop_IT(htim);
    spi4_state_machine_update(NEXT);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if(hspi == &hspi4) {
        spi4_state_machine_update(NEXT);
    }
}