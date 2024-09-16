#include "radio.h"

#define SPI_TIMEOUT 1000

void sx1262_hal_write(uint8_t* tx_buffer, const uint16_t length)
{
  HAL_SPI_Transmit(&hspi2, tx_buffer, length, SPI_TIMEOUT);
}

void sx1262_hal_read(uint8_t* rx_buffer, const uint16_t length)
{
  HAL_SPI_Receive(&hspi2, rx_buffer, length, SPI_TIMEOUT);
}

void sx1262_hal_read_write(uint8_t* tx_buffer, uint8_t* rx_buffer, const uint16_t length)
{
  HAL_SPI_TransmitReceive(&hspi2, tx_buffer, rx_buffer, length, SPI_TIMEOUT);
}

void sx1262_hal_set_cs(uint8_t pin_state)
{
  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, pin_state);
}

bool sx1262_hal_read_irq()
{
  return HAL_GPIO_ReadPin(LORA_IRQ_GPIO_Port, LORA_IRQ_Pin);
}

void sx1262_hal_reset()
{
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
}

void sx1262_hal_delay(uint32_t delay) 
{
  HAL_Delay(delay);
}

uint32_t sx1262_hal_millis()
{
  return HAL_GetTick();
}