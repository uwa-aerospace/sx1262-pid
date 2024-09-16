/*
 * sx1262_hal.h
 * PID (platform independent driver) for the SX1262 LoRa module, written in C
 * Port of: https://github.com/thekakester/Arduino-LoRa-Sx1262/
 * 
 * Author: txvvgnx
 * Original library by: thekakester
*/

#ifndef __SX1262_HAL_H
#define __SX1262_HAL_H

/* INCLUDES */ 
#include <stdint.h>
#include <stdbool.h>

/* FUNCTIONS */
/**
 * SPI Radio data transfer - write
 *
 * @remark Shall be implemented by the user
 *
 * @param tx_buffer  Pointer to the buffer to be transmitted
 * @param length     Buffer size to be transmitted
 *
 */
void sx1262_hal_write(uint8_t* tx_buffer, const uint16_t length);

/**
 * SPI Radio data transfer - read
 *
 * @remark Shall be implemented by the user
 *
 * @param rx_buffer  Pointer to the buffer to be transmitted
 * @param length     Buffer size to be transmitted
 *
 */
void sx1262_hal_read(uint8_t* rx_buffer, const uint16_t length);

/**
 * SPI Radio data transfer - read and write simultaneously
 *
 * @remark Shall be implemented by the user
 *
 * @param tx_buffer   Pointer to the buffer to be transmitted
 * @param rx_buffer   Pointer to the buffer to be received
 * @param length      Buffer size to be transmitted and received - number of bytes sent and received is equal
 *
 */
void sx1262_hal_read_write(uint8_t* tx_buffer, uint8_t* rx_buffer, const uint16_t length);

/**
 * Sets the chip select (CS) pin to LOW (0) or HIGH (1)
 *
 * @remark Shall be implemented by the user
 *
 * @param pin_state    LOW(0) or HIGH(1)
 *
 */
void sx1262_hal_set_cs(uint8_t pin_state);

/**
 * Reads the value of the interrupt (IRQ) pin
 *
 * @remark Shall be implemented by the user
 *
 * @returns Value of the IRQ pin, 0 (LOW) or 1 (HIGH)
 *
 */
bool sx1262_hal_read_irq();

/**
 * Reset the radio
 *
 * @remark Shall be implemented by the user
 *
 */
void sx1262_hal_reset();

/**
 * Wait for a number of miliseconds
 *
 * @remark Shall be implemented by the user
 *
 * @param delay   Delay in milliseconds
 * 
 */
void sx1262_hal_delay(uint32_t delay);

/**
 * Get the number of milliseconds since boot
 * 
 * @remark Shall be implemented by the user
 * 
 * @returns Number of milliseconds elapsed since boot
 */
uint32_t sx1262_hal_millis();

#endif