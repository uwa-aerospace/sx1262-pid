/*
 * sx1262.c
 * PID (platform independent driver) for the SX1262 LoRa module, written in C
 * Port of: https://github.com/thekakester/Arduino-LoRa-Sx1262/
 * 
 * Author: txvvgnx
 * Original library by: thekakester
*/

#include "sx1262.h"
#include "sx1262_hal.h"

/* MACROS */
#define SX1262_NOP 0x00 // Write this to SPI bus while reading data, or as a dummy/placeholder
#define DEFAULT_FREQUENCY 915000000 // 915 MHz

/* VARIABLES */
static bool IN_RECEIVE_MODE = false;
static uint8_t spi_buffer[32];

/* CONFIG VARIABLES */
static uint32_t pll_frequency;
static uint8_t bandwidth;
static uint8_t coding_rate;
static uint8_t spreading_factor;
static uint8_t low_data_rate_optimize;
static uint32_t transmit_timeout;

/* RX STAT VARIABLES */
static int rssi;
static int snr;
static int signal_rssi;

/* COMMANDS */
typedef enum sx1262_commands_e 
{
// Operational Modes Functions
  SX1262_SET_SLEEP                  = 0x84,
  SX1262_SET_STANDBY                = 0x80,
  SX1262_SET_FS                     = 0xC1,
  SX1262_SET_TX                     = 0x83,
  SX1262_SET_RX                     = 0x82,
  SX1262_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
  SX1262_SET_PA_CFG                 = 0x95,

  // Register and Buffer Access
  SX1262_WRITE_REGISTER = 0x0D,
  SX1262_READ_REGISTER  = 0x1D,
  SX1262_WRITE_BUFFER   = 0x0E,
  SX1262_READ_BUFFER    = 0x1E,

  // DIO and IRQ commands
  SX1262_SET_DIO_IRQ_PARAMS         = 0x08,
  SX1262_CLR_IRQ_STATUS             = 0x02,
  SX1262_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,

  // RF Modulation and Packet-Related Functions
  SX1262_SET_RF_FREQUENCY          = 0x86,
  SX1262_SET_PKT_TYPE              = 0x8A,
  SX1262_SET_TX_PARAMS             = 0x8E,
  SX1262_SET_MODULATION_PARAMS     = 0x8B,
  SX1262_SET_PKT_PARAMS            = 0x8C,
  SX1262_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,

  // Communication Status Information
  SX1262_GET_STATUS           = 0xC0,
  SX1262_GET_PKT_STATUS       = 0x14,
  SX1262_GET_RX_BUFFER_STATUS = 0x13,
} sx1262_commands_t;

/* FUNCTION DEFINITIONS */

/** Initializes the radio and sets default parameters. */
bool sx1262_setup()
{
  sx1262_hal_set_cs(1); // Set CS to high to solve undefined behavior
  
  sx1262_hal_reset(); // Hardware reset the chip by toggling the NRST pin

  if (!sx1262_sanity_check()) // Verify that the SPI connection is working properly
    return false;

  sx1262_configure_radio();

  return true;
}

/**
 * Verifies that the SPI connection is working properly.
 * @returns True if radio is communicating over SPI. False if no connection.
 */
bool sx1262_sanity_check()
{
  uint16_t address = 0x0740; // LoRa sync word MSB register

  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_READ_REGISTER;
  spi_buffer[1] = (address >> 8) & 0xFF;
  spi_buffer[2] = (address >> 0) & 0xFF;
  spi_buffer[3] = 0x00;
  spi_buffer[4] = 0x00;
  sx1262_hal_read_write(spi_buffer, spi_buffer, 5);
  sx1262_hal_set_cs(1);

  return spi_buffer[4] == 0x14; // Register should have a value of 0x14 after reset
}

/** Send the bare-bones required commands needed for radio to run. */ 
void sx1262_configure_radio()
{
  // Set DIO2 as RF switch
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_DIO2_AS_RF_SWITCH_CTRL;
  spi_buffer[1] = 0x01;
  sx1262_hal_write(spi_buffer, 2);
  sx1262_hal_set_cs(1);
  sx1262_hal_delay(100); // Give radio some time to process the command

  // Set radio frequency
  sx1262_set_frequency(DEFAULT_FREQUENCY);

  // Set modem to LoRa
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_PKT_TYPE;
  spi_buffer[1] = 0x01;  // Packet Type: 0x00=GFSK, 0x01=LoRa
  sx1262_hal_write(spi_buffer, 2);
  sx1262_hal_set_cs(1);
  sx1262_hal_delay(100); // Give radio some time to process the command

  // Set Rx Timeout to reset on SyncWord or Header detection
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_STOP_TIMER_ON_PREAMBLE;
  spi_buffer[1] = 0x00; // Stop timer on:  0x00=SyncWord or header detection, 0x01=preamble detection
  sx1262_hal_write(spi_buffer, 2);
  sx1262_hal_set_cs(1);
  sx1262_hal_delay(100); // Give radio some time to process the command

  sx1262_set_preset(PRESET_DEFAULT); // Sets default modulation parameters

  // Set PA Config
  // See datasheet 13.1.4 for descriptions and optimal settings recommendations
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_PA_CFG;  //Opcode for "SetPaConfig"
  spi_buffer[1] = 0x04;               //paDutyCycle. See datasheet, set in conjuntion with hpMax
  spi_buffer[2] = 0x07;               //hpMax.  Basically Tx power.  0x00-0x07 where 0x07 is max power
  spi_buffer[3] = 0x00;               //device select: 0x00 = SX1262, 0x01 = SX1261
  spi_buffer[4] = 0x01;               //paLut (reserved, always set to 1)
  sx1262_hal_write(spi_buffer, 5);
  sx1262_hal_set_cs(1);
  sx1262_hal_delay(100); // Give radio some time to process the command

  // Set TX Params
  // See datasheet 13.4.4 for details
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_TX_PARAMS; //Opcode for SetTxParams
  spi_buffer[1] = 22;                   //Power.  Can be -17(0xEF) to +14x0E in Low Pow mode.  -9(0xF7) to 22(0x16) in high power mode
  spi_buffer[2] = 0x02;                 //Ramp time. Lookup table.  See table 13-41. 0x02="40uS"
  sx1262_hal_write(spi_buffer, 3);
  sx1262_hal_set_cs(1);
  sx1262_hal_delay(100); // Give radio some time to process the command

  //Set LoRa Symbol Number timeout
  //How many symbols are needed for a good receive.
  //Symbols are preamble symbols
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_LORA_SYMB_NUM_TIMEOUT;  //Opcode for "SetLoRaSymbNumTimeout"
  spi_buffer[1] = 0x00;                              //Number of symbols.  Ping-pong example from Semtech uses 5
  sx1262_hal_write(spi_buffer, 2);
  sx1262_hal_set_cs(1);
  sx1262_hal_delay(100); // Give radio some time to process the command

  //Enable interrupts
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_DIO_IRQ_PARAMS;  //0x08 is the opcode for "SetDioIrqParams"
  spi_buffer[1] = 0x00;                       //IRQMask MSB. IRQMask is "what interrupts are enabled"
  spi_buffer[2] = 0x02;                       //IRQMask LSB. See datasheet table 13-29 for details
  spi_buffer[3] = 0xFF;                       //DIO1 mask MSB. Of the interrupts detected, which should be triggered on DIO1 pin
  spi_buffer[4] = 0xFF;                       //DIO1 Mask LSB
  spi_buffer[5] = 0x00;                       //DIO2 Mask MSB
  spi_buffer[6] = 0x00;                       //DIO2 Mask LSB
  spi_buffer[7] = 0x00;                       //DIO3 Mask MSB
  spi_buffer[8] = 0x00;                       //DIO3 Mask LSB
  sx1262_hal_write(spi_buffer, 9);
  sx1262_hal_set_cs(1);
  sx1262_hal_delay(100); // Give radio some time to process the command
}

/** Transmit a block of data 
 * @param data A string to transmit, maximum length of 255 characters.
 * @param data_length Length of the string to transmit. 
 * */ 
void sx1262_transmit(char* data, uint8_t data_length)
{
  // Max lora packet size is 255 bytes
  if (data_length > 255)
    data_length = 255;

  if (IN_RECEIVE_MODE)
    sx1262_set_mode_standby();

  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_PKT_PARAMS;  //Opcode for "SetPacketParameters"
  spi_buffer[1] = 0x00;                   //PacketParam1 = Preamble Len MSB
  spi_buffer[2] = 0x0C;                   //PacketParam2 = Preamble Len LSB
  spi_buffer[3] = 0x00;                   //PacketParam3 = Header Type. 0x00 = Variable Len, 0x01 = Fixed Length
  spi_buffer[4] = data_length;            //PacketParam4 = Payload Length (Max is 255 bytes)
  spi_buffer[5] = 0x01;                   //PacketParam5 = CRC Type. 0x00 = Off, 0x01 = on
  spi_buffer[6] = 0x00;                   //PacketParam6 = Invert IQ.  0x00 = Standard, 0x01 = Inverted
  sx1262_hal_write(spi_buffer, 7);
  sx1262_hal_set_cs(1);
  sx1262_wait_for_command_completion(100); // Give radio 100ms to process the command

  // Write payload to the buffer
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_WRITE_BUFFER,  //Opcode for WriteBuffer command
  spi_buffer[1] = 0x00;                 //Dummy byte before writing payload
  sx1262_hal_write(spi_buffer, 2);

  // Write the buffer 32 bytes at a time
  uint8_t size = sizeof(spi_buffer);
  for (uint16_t i = 0; i < data_length; i += size) {
    if (i + size > data_length) 
      size = data_length - i;

    memcpy(spi_buffer, &(data[i]), size);
    sx1262_hal_write(spi_buffer, size);
  }
  sx1262_hal_set_cs(1);
  sx1262_wait_for_command_completion(1000); // Give radio 1s to process the command

  // Transmit
  // An interrupt will be triggered if we surpass our timeout
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_TX;  //Opcode for SetTx command
  spi_buffer[1] = 0xFF;           //Timeout (3-byte number)
  spi_buffer[2] = 0xFF;           //Timeout (3-byte number)
  spi_buffer[3] = 0xFF;           //Timeout (3-byte number)
  sx1262_hal_write(spi_buffer, 4);
  sx1262_hal_set_cs(1);
  sx1262_wait_for_command_completion(transmit_timeout); //Wait for tx to complete, with a timeout so we don't wait forever

  //Remember that we are in Tx mode.  If we want to receive a packet, we need to switch into receiving mode
  IN_RECEIVE_MODE = false;
}

/**
 * Set radio into standby mode.
 * @remark Switching directly from Rx to Tx mode can be slow, so we first want to go into standby
 */ 
void sx1262_set_mode_standby() 
{
  // Tell the chip to wait for it to receive a packet.
  // Based on our previous config, this should throw an interrupt when we get a packet
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_STANDBY; //0x80 is the opcode for "SetStandby"
  spi_buffer[1] = 0x01;               //0x00 = STDBY_RC, 0x01=STDBY_XOSC
  sx1262_hal_write(spi_buffer, 2);
  sx1262_hal_set_cs(1);
  sx1262_wait_for_command_completion(100);

  IN_RECEIVE_MODE = false;  //No longer in receive mode
}

/** Sets the radio into receive mode, allowing it to listen for incoming packets.
 * @remark If radio is already in receive mode, this does nothing.
 */
void sx1262_set_mode_receive()
{
  if (IN_RECEIVE_MODE) return; //We're already in receive mode, this would do nothing

  //Set packet parameters
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_PKT_PARAMS;  //Opcode for "SetPacketParameters"
  spi_buffer[1] = 0x00;                   //PacketParam1 = Preamble Len MSB
  spi_buffer[2] = 0x0C;                   //PacketParam2 = Preamble Len LSB
  spi_buffer[3] = 0x00;                   //PacketParam3 = Header Type. 0x00 = Variable Len, 0x01 = Fixed Length
  spi_buffer[4] = 0xFF;                   //PacketParam4 = Payload Length (Max is 255 bytes)
  spi_buffer[5] = 0x00;                   //PacketParam5 = CRC Type. 0x00 = Off, 0x01 = on
  spi_buffer[6] = 0x00;                   //PacketParam6 = Invert IQ.  0x00 = Standard, 0x01 = Inverted
  sx1262_hal_write(spi_buffer, 7);
  sx1262_hal_set_cs(1);
  sx1262_wait_for_command_completion(100);

  // Tell the chip to wait for it to receive a packet.
  // Based on our previous config, this should throw an interrupt when we get a packet
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_RX; //0x82 is the opcode for "SetRX"
  spi_buffer[1] = 0xFF;          //24-bit timeout, 0xFFFFFF means no timeout
  spi_buffer[2] = 0xFF;          // ^^
  spi_buffer[3] = 0xFF;          // ^^
  sx1262_hal_write(spi_buffer, 4);
  sx1262_hal_set_cs(1);
  sx1262_wait_for_command_completion(100);

  //Remember that we're in receive mode so we don't need to run this code again unnecessarily
  IN_RECEIVE_MODE = true;
}

/**
 * Receive a packet if available
 * @returns 
 *    - -1 when no packet is available.
 * 
 *    - 0 when an empty packet is received (packet with no payload).
 *  
 *    - Payload size (1-255) when a packet with a non-zero payload is received.
*/
int sx1262_receive_async(char* buff, uint8_t buff_max_len)
{
  sx1262_set_mode_receive();

  if (!sx1262_hal_read_irq())
    return -1; //Return -1, meanining no packet ready

  //Tell the radio to clear the interrupt, and set the pin back inactive.
  while (sx1262_hal_read_irq()) {
    //Clear all interrupt flags.  This should result in the interrupt pin going low
    sx1262_hal_set_cs(0);
    spi_buffer[0] = SX1262_CLR_IRQ_STATUS;  //Opcode for ClearIRQStatus command
    spi_buffer[1] = 0xFF;                   //IRQ bits to clear (MSB) (0xFFFF means clear all interrupts)
    spi_buffer[2] = 0xFF;                   //IRQ bits to clear (LSB)
    sx1262_hal_write(spi_buffer, 3);
    sx1262_hal_set_cs(1);
  }

  // (Optional) Read the packet status info from the radio.
  // This is things like radio strength, noise, etc.
  // See datasheet 13.5.3 for more info
  // This provides debug info about the packet we received
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_GET_PKT_STATUS;  //Opcode for get packet status
  spi_buffer[1] = 0xFF;                   //Dummy byte. Returns status
  spi_buffer[2] = 0xFF;                   //Dummy byte. Returns rssi
  spi_buffer[3] = 0xFF;                   //Dummy byte. Returns snd
  spi_buffer[4] = 0xFF;                   //Dummy byte. Returns signal RSSI
  sx1262_hal_read_write(spi_buffer, spi_buffer, 5);
  sx1262_hal_set_cs(1);

  //Store these values as class variables so they can be accessed if needed
  //Documentation for what these variables mean can be found in the .h file
  rssi        = -((int)spi_buffer[2]) / 2;  //"Average over last packet received of RSSI. Actual signal power is –RssiPkt/2 (dBm)"
  snr         =  ((int8_t)spi_buffer[3]) / 4;   //SNR is returned as a SIGNED byte, so we need to do some conversion first
  signal_rssi = -((int)spi_buffer[4]) / 2;

  //We're almost ready to read the packet from the radio
  //But first we have to know how big the packet is, and where in the radio memory it is stored
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_GET_RX_BUFFER_STATUS; //Opcode for GetRxBufferStatus command
  spi_buffer[1] = 0xFF;                        //Dummy.  Returns radio status
  spi_buffer[2] = 0xFF;                        //Dummy.  Returns loraPacketLength
  spi_buffer[3] = 0xFF;                        //Dummy.  Returns memory offset (address)
  sx1262_hal_read_write(spi_buffer, spi_buffer, 4);
  sx1262_hal_set_cs(1);

  uint8_t payload_len = spi_buffer[2];    //How long the lora packet is
  uint8_t start_address = spi_buffer[3];  //Where in 1262 memory is the packet stored
  //Make sure we don't overflow the buffer if the packet is larger than our buffer
  if (buff_max_len < payload_len) 
    payload_len = buff_max_len;

  //Read the radio buffer from the SX1262 into the user-supplied buffer
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_READ_BUFFER; //Opcode for ReadBuffer command
  spi_buffer[1] = start_address;      //SX1262 memory location to start reading from
  spi_buffer[2] = 0x00;               //Dummy byte
  sx1262_hal_write(spi_buffer, 3);
  sx1262_hal_read(buff, payload_len);
  sx1262_hal_set_cs(1);

  return payload_len;  //Return how many bytes we actually read
}

/** Get the Received Strength Signal Indicator (RSSI) of an incoming radio signal
 * @returns Received Strength Signal Indicator (RSSI), in dBm (decibel-milliwatts)
 */
int sx1262_get_rssi()
{
  return rssi;
}

/** Get the Signal-to-Noise Ratio (SNR) of an incoming radio signal
 * @returns Received Signal-to-Noise Ratio (SNR), in dB (decibels)
 */
int sx1262_get_snr()
{
  return snr;
}

/** Get the RSSI of an incoming radio signal without accounting for noise
 * @returns Received Signal RSSI (without noise), in dBm (decibel-milliwatts)
 * @remark If the SNR is positive then there should be no difference between RSSI and Signal RSSI
 */
int sx1262_get_signal_rssi()
{
  return signal_rssi;
}

/**
 * This command will wait until the radio reports that it is no longer busy.
 * This is useful when waiting for commands to finish that take a while such as transmitting packets.
 * @param timeout Timeout in milliseconds - avoids an infinite loop if something happens to the radio
 * @returns TRUE on success, FALSE if timeout hit
*/
bool sx1262_wait_for_command_completion(uint32_t timeout)
{
  uint32_t start_time = sx1262_hal_millis();
  bool data_transmitted = false;

  while (!data_transmitted)
  {
    // Wait some time between spamming SPI status commands, asking if the chip is ready yet.
    // Some commands take a bit before the radio even changes into a busy state,
    // so if we check too fast we might pre-maturely think we're done processing the command

    // 3ms delay gives inconsistent results.  4ms seems stable.  Using 5ms to be safe
    sx1262_hal_delay(5);

    sx1262_hal_set_cs(0);
    spi_buffer[0] = SX1262_GET_STATUS;  //Opcode for "getStatus" command
    spi_buffer[1] = SX1262_NOP;         //Dummy byte, status will overwrite this byte
    uint8_t rx_buffer[2];
    sx1262_hal_read_write(spi_buffer, spi_buffer, 2);
    sx1262_hal_set_cs(1);

    // Parse out the status (see datasheet for what each bit means)
    uint8_t chip_mode = (spi_buffer[1] >> 4) & 0x7;       //Chip mode is bits [6:4] (3-bits)
    uint8_t command_status = (spi_buffer[1] >> 1) & 0x7;  //Command status is bits [3:1] (3-bits)

    //Status 0, 1, 2 mean we're still busy.  Anything else means we're done.
    //Commands 3-6 = command timeout, command processing error, failure to execute command, and Tx Done (respoectively)
    if (command_status != 0 && command_status != 1 && command_status != 2) {
      data_transmitted = true;
    }

    //If we're in standby mode, we don't need to wait at all
    //0x03 = STBY_XOSC, 0x02= STBY_RC
    if (chip_mode == 0x03 || chip_mode == 0x02) {
      data_transmitted = true;
    }

    //Avoid infinite loop by implementing a timeout
    if (sx1262_hal_millis() - start_time >= timeout) {
      return false;
    }
  }
  
  return true;
}

/** 
 * Set the radio modulation parameters.
 * @remark This is broken into its own function because this command might get called frequently
 */
void sx1262_update_modulation_parameters()
{
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_MODULATION_PARAMS;  //Opcode for "SetModulationParameters"
  spi_buffer[1] = spreading_factor;              //ModParam1 = Spreading Factor.  Can be SF5-SF12, written in hex (0x05-0x0C)
  spi_buffer[2] = bandwidth;                     //ModParam2 = Bandwidth.  See Datasheet 13.4.5.2 for details. 0x00=7.81khz (slowest)
  spi_buffer[3] = coding_rate;                   //ModParam3 = CodingRate.  Semtech recommends CR_4_5 (which is 0x01).
  spi_buffer[4] = low_data_rate_optimize;        //LowDataRateOptimize.  0x00 = 0ff, 0x01 = On.  Required to be on for SF11 + SF12
  sx1262_hal_write(spi_buffer, 5);
  sx1262_hal_set_cs(1);
  sx1262_hal_delay(100); // Give radio some time to process the command

switch (spreading_factor) {
    case 12:
      transmit_timeout = 252000; //Actual tx time 126 seconds
      break;
    case 11:
      transmit_timeout = 160000; //Actual tx time 81 seconds
      break;
    case 10:
      transmit_timeout = 60000; //Actual tx time 36 seconds
      break;
    case 9:
      transmit_timeout = 40000; //Actual tx time 20 seconds
      break;
    case 8:
      transmit_timeout = 20000; //Actual tx time 11 seconds
      break;
    case 7:
      transmit_timeout = 12000; //Actual tx time 6.3 seconds
      break;
    case 6:
      transmit_timeout = 7000; //Actual tx time 3.7s seconds
      break;
    default:  //SF5
      transmit_timeout = 5000; //Actual tx time 2.2 seconds
      break;
  }
}

/*
* Argument: pass in one of the following
*     - PRESET_DEFAULT:   Default radio config.
*                         Medium range, medium speed
*     - PRESET_FAST:      Faster speeds, but less reliable at long ranges.
*                         Use when you need fast data transfer and have radios close together
*     - PRESET_LONGRANGE: Most reliable option, but slow. Suitable when you prioritize
*                         reliability over speed, or when transmitting over long distances
*/
bool sx1262_set_preset(uint8_t preset) 
{
  if (preset == PRESET_DEFAULT) {
    bandwidth = 5;               //250khz
    coding_rate = 1;             //CR_4_5
    spreading_factor = 7;        //SF7
    low_data_rate_optimize = 0;  //Don't optimize (used for SF12 only)
    sx1262_update_modulation_parameters();
    return true;
  }

  if (preset == PRESET_LONGRANGE) {
    bandwidth = 4;               //125khz
    coding_rate = 1;             //CR_4_5
    spreading_factor = 12;       //SF12
    low_data_rate_optimize = 1;  //Optimize for low data rate (SF12 only)
    sx1262_update_modulation_parameters();
    return true;
  }

  if (preset == PRESET_FAST) {
    bandwidth = 6;               //500khz
    coding_rate = 1;             //CR_4_5
    spreading_factor = 5;        //SF5
    low_data_rate_optimize = 0;  //Don't optimize (used for SF12 only)
    sx1262_update_modulation_parameters();
    return true;
  }

  //Invalid preset specified
  return false;
}

/**
  * Set the bandwith (basically, this is how big the frequency span is that we occupy)
  * 
  * - Bigger bandwidth allows us to transmit large amounts of data faster, but it occupies a larger span of frequencies.
  * 
  * - Smaller bandwidth takes longer to transmit large amounts of data, but its less likely to collide with other frequencies.
  *
  * @remark Available bandwidth settings, pulled from datasheet 13.4.5.2
  *  SETTING.   | Bandwidth
  * ------------+-----------
  *    0x00     |    7.81khz
  *    0x08     |   10.42khz
  *    0x01     |   15.63khz
  *    0x09     |   20.83khz
  *    0x02     |   31.25khz
  *    0x0A     |   41.67khz
  *    0x03     |   62.50khz
  *    0x04     |  125.00khz
  *    0x05     |  250.00khz (default)
  *    0x06     |  500.00khz
  *
  * @returns TRUE on success, FALSE on failure (invalid bandwidth)
  */
bool sx1262_set_bandwidth(uint8_t new_bandwidth)
{
  //Bandwidth setting must be 0-10 (excluding 7 for some reason)
  if (new_bandwidth < 0 || new_bandwidth > 0x0A || new_bandwidth == 7) 
    return false;

  bandwidth = new_bandwidth;
  sx1262_update_modulation_parameters();

  return true;
}

/**
  * Coding rate affects FEC (Foward Error Correction). A higher coding rate provides better immunity to noise.
  * @remark
  *  SETTING  | Coding Rate
  * ----------+--------------------
  *    0x01   |   CR_4_5 (default)
  *    0x02   |   CR_4_6
  *    0x03   |   CR_4_7
  *    0x04   |   CR_4_8
  *
  * @returns TRUE on success, FALSE on failure (invalid coding rate)
*/
bool sx1262_set_coding_rate(uint8_t new_coding_rate)
{
  //Coding rate must be 1-4 (inclusive)
  if (new_coding_rate < 1 || new_coding_rate > 4) 
    return false;

  coding_rate = new_coding_rate;
  sx1262_update_modulation_parameters();

  return true;
}

/**
  * Change the spreading factor of a packet
  *
  * - High spreading factors are good for longer distances with slower transmit speeds.
  * 
  * - Low spreading factors are good when the radios are close, which allows faster transmission speeds.
  *
  * @remark
  * Setting | Spreading Factor
  * --------+---------------------------
  *    5    | SF5 (fastest, short range)
  *    6    | SF6
  *    7    | SF7 (default)
  *    8    | SF8
  *    9    | SF9
  *   10    | SF10 
  *   11    | SF11
  *   12    | SF12 (Slowest, long range, most reliable)
  *
  * @returns TRUE on success, FALSE on failure (invalid spreading factor)
  */
bool sx1262_set_spreading_factor(uint8_t new_spreading_factor)
{
  if (new_spreading_factor < 5 || new_spreading_factor > 12) 
    return false;

  // The datasheet highly recommends enabling "LowDataRateOptimize" for SF11 and SF12
  low_data_rate_optimize = (new_spreading_factor >= 11) ? 1 : 0;  //Turn on for SF11+SF12, turn off for anything else
  spreading_factor = new_spreading_factor;

  sx1262_update_modulation_parameters();

  return true;
}

/**
 * Set the operating frequency of the radio.
 * @returns True if frequency was set successfully, False if frequency is invalid.
 */
bool sx1262_set_frequency(uint32_t frequency_hz)
{
  if (frequency_hz < 150000000 || frequency_hz > 960000000) { return false; }

  pll_frequency = sx1262_frequency_to_pll(frequency_hz);
  sx1262_update_radio_frequency();

  return true;
}

/**
 * Set the radio frequency.
 * @remark pll_frequency must be set before calling this function
 */
void sx1262_update_radio_frequency()
{
  sx1262_hal_set_cs(0);
  spi_buffer[0] = SX1262_SET_RF_FREQUENCY;
  spi_buffer[1] = (pll_frequency >> 24) & 0xFF;
  spi_buffer[2] = (pll_frequency >> 16) & 0xFF;
  spi_buffer[3] = (pll_frequency >> 8) & 0xFF;
  spi_buffer[4] = (pll_frequency >> 0) & 0xFF;
  sx1262_hal_write(spi_buffer, 5);
  sx1262_hal_set_cs(1);
  sx1262_hal_delay(100);
}

/** Convert a frequency in Hz (such as 915000000) to the respective PLL setting
 * @remark Assumes a 32MHz clock
 * @param freq_in_hz
 */
uint32_t sx1262_frequency_to_pll(uint32_t freq_in_hz)
{
  uint32_t q = freq_in_hz / 15625UL;  //Gives us the result (quotient), rounded down to the nearest integer
  uint32_t r = freq_in_hz % 15625UL;  //Everything that isn't divisible, aka "the part that hasn't been divided yet"

  //Multiply by 16384 to satisfy the equation above
  q *= 16384UL;
  r *= 16384UL; //Don't forget, this part still needs to be divided because it was too small to divide before
  
  return q + (r / 15625UL);  //Finally divide the the remainder part before adding it back in with the quotient
}
