## Important note
In order for this driver to work, you must implement the following *platform-dependent* HAL functions:
- sx1262_hal_write
- sx1262_hal_read
- sx1262_hal_read_write
- sx1262_hal_set_cs
- sx1262_hal_read_irq
- sx1262_hal_reset
- sx1262_hal_delay
- sx1262_hal_millis

An example HAL implementation for `STM32` has been provided.

## Usage
### Transmit example:
```c
void main()
{
    // Set the radio up
    if (!sx1262_setup()) {
        print_console("Failed to initialize radio\n");
    }

    char* message = "Hello World";

    // Transmit "Hello World" indefinitely
    for(;;)
    {
        print_console("Transmitting... ");
        sx1262_transmit(message, strlen(message));
        delay(500);
    }
}
```

### Receive example:
```c
void main()
{
    // Set the radio up
    if (!sx1262_setup()) {
        print_console("Failed to initialize radio\n");
    }

    char receiveBuffer[255];

    // Transmit "Hello World" indefinitely
    for(;;)
    {
        int bytesRead = sx1262_receive_async(receiveBuffer, sizeof(receiveBuffer));

        if (bytesRead > -1) {
        // Add a null terminator at the end of the valid data
        if (bytesRead < sizeof(receiveBuffer)) {
            receiveBuffer[bytesRead] = '\0';  // Ensure the string is null-terminated
        } 
        else {
            // If bytesRead == sizeof(receiveBuffer), avoid writing out of bounds
            receiveBuffer[sizeof(receiveBuffer) - 1] = '\0';
        }

        // Print the payload over serial
        print_console("Received %d bytes: %s\n", bytesRead, receiveBuffer);
        print_console("RSSI: %d, SNR: %d, Signal RSSI: %d\n", sx1262_get_rssi(), sx1262_get_snr(), sx1262_get_signal_rssi());
        }
    }
}
```