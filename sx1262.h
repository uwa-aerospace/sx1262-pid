/*
 * sx1262.h
 * PID (platform independent driver) for the SX1262 LoRa module, written in C
 * Port of: https://github.com/thekakester/Arduino-LoRa-Sx1262/
 * 
 * Author: txvvgnx
 * Original library by: thekakester
*/

#ifndef __SX1262_H
#define __SX1262_H

/* INCLUDES */ 
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* PRESETS */
#define PRESET_DEFAULT 0
#define PRESET_LONGRANGE 1
#define PRESET_FAST 2

/* FUNCTIONS */
bool sx1262_setup();
bool sx1262_sanity_check();
void sx1262_transmit(char* data, uint8_t data_length);
int sx1262_receive_async(char* buff, uint8_t buff_max_len);

bool sx1262_set_preset(uint8_t preset);
bool sx1262_set_frequency(uint32_t frequency_hz);
bool sx1262_set_bandwidth(uint8_t new_bandwidth);
bool sx1262_set_coding_rate(uint8_t new_coding_rate);
bool sx1262_set_spreading_factor(uint8_t new_spreading_factor);

int sx1262_get_rssi();
int sx1262_get_snr();
int sx1262_get_signal_rssi();

uint32_t sx1262_frequency_to_pll(uint32_t freq_in_hz);

void sx1262_set_mode_receive();
void sx1262_set_mode_standby();
bool sx1262_wait_for_command_completion(uint32_t timeout);

void sx1262_configure_radio();
void sx1262_update_radio_frequency();
void sx1262_update_modulation_parameters();

#endif