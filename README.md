# ESP32 Bare-Metal SPI Driver & Sensor Compensation for BMP280

Production-grade, bare-metal firmware implementation of full-duplex **Serial Peripheral Interface (SPI)** communication between an **ESP32 (Master)** and a **Bosch BMP280 digital pressure and temperature sensor (Slave)**. 

This project goes beyond high-level library abstraction, focusing on low-level register configuration, bus timing analysis, multi-byte burst transactions, and proprietary fixed-point compensation mathematics.

---

## 🛠️ Hardware Architecture & Pinout

The system utilizes a 4-wire synchronous full-duplex SPI bus operating in **Mode 0** (CPOL = 0, CPHA = 0).

| ESP32 Pin | BMP280 Pin | Function | Description |
| :--- | :--- | :--- | :--- |
| **GPIO 23** | SDI (MOSI) | Master Out, Slave In | Command and address transmission |
| **GPIO 19** | SDO (MISO) | Master In, Slave Out | Register data streaming |
| **GPIO 18** | SCK (SCLK) | Serial Clock | Synchronous clock generation  |
| **GPIO 5**  | CSB (CS)   | Chip Select | Active-LOW slave arbitration line |
| **3.3V**    | VCC / VIO  | Power Supply | Analog/Digital power rail |
| **GND**     | GND        | Ground | Common system ground reference |

---

## ⚡ Core Technical Highlights

### 1. Read/Write Arbitration via MSB Manipulation
The BMP280 protocol encodes register access direction directly into the Most Significant Bit (MSB) of the address byte:
* **Write Operation (Bit 7 = 0):** Target address is modified (e.g., control register `0xF4` becomes `0x74`) to instruct the slave to accept incoming configuration bytes.
* **Read Operation (Bit 7 = 1):** Target address has bit 7 set high (e.g., calibration base `0x88` or data output `0xF7`) to initiate data streaming on MISO.

### 2. Burst Transactions & High-Impedance (Hi-Z) State
* **Multi-Byte Streaming:** Holding the Chip Select (CS) line LOW while maintaining active clock pulses allows sequential reads of memory registers (e.g., retrieving 24 bytes of calibration coefficients or 7 bytes of live sensor data in a single transaction).
* **Bus Contention Prevention:** Unselected slaves maintain their MISO output driver in a **High-Impedance (Hi-Z / Tri-state)** state, preventing electrical collisions on shared multi-slave topologies.

### 3. Clock Synchronization (CPOL & CPHA)
* Configured for **Mode 0** (CPOL = 0, CPHA = 0): The clock idles LOW and data is sampled on the leading rising edge.
* Fully compatible with **Mode 3** (CPOL = 1, CPHA = 1) due to the physical symmetry of rising-edge data sampling.

---

## 💻 Firmware Implementation

The complete bare-metal implementation written in **ESP-IDF C**:

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

void app_main(void)
{
    // 1. Bus Configuration
    spi_bus_config_t buscfg = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // 2. Device Configuration (SPI Mode 0, 1 MHz)
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = 5,
        .queue_size = 7
    };

    // 3. Initialize SPI Host and Add Device
    spi_device_handle_t spi;
    spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI3_HOST, &devcfg, &spi);

    // 4. Wake Sensor: Write 0x27 (Normal Mode) to Control Register 0x74
    uint8_t tx_wake[2] = {0x74, 0x27};
    spi_transaction_t wake_trans = { .length = 16, .tx_buffer = tx_wake };
    spi_device_polling_transmit(spi, &wake_trans);

    // 5. Read Factory Calibration Data (24 bytes starting at 0x88)
    uint8_t tx_calib[25] = {0x88}; 
    uint8_t rx_calib[25] = {0};
    spi_transaction_t calib_trans = { .length = 200, .tx_buffer = tx_calib, .rx_buffer = rx_calib };
    spi_device_polling_transmit(spi, &calib_trans);

    // Reconstruct Temperature & Pressure Calibration Coefficients
    uint16_t dig_T1 = (rx_calib[2] << 8) | rx_calib[1];
    int16_t  dig_T2 = (int16_t)((rx_calib[4] << 8) | rx_calib[3]);
    int16_t  dig_T3 = (int16_t)((rx_calib[6] << 8) | rx_calib[5]);

    uint16_t dig_P1 = (rx_calib[8] << 8)  | rx_calib[7];
    int16_t  dig_P2 = (int16_t)((rx_calib[10] << 8) | rx_calib[9]);
    int16_t  dig_P3 = (int16_t)((rx_calib[12] << 8) | rx_calib[11]);
    int16_t  dig_P4 = (int16_t)((rx_calib[14] << 8) | rx_calib[13]);
    int16_t  dig_P5 = (int16_t)((rx_calib[16] << 8) | rx_calib[15]);
    int16_t  dig_P6 = (int16_t)((rx_calib[18] << 8) | rx_calib[17]);
    int16_t  dig_P7 = (int16_t)((rx_calib[20] << 8) | rx_calib[19]);
    int16_t  dig_P8 = (int16_t)((rx_calib[22] << 8) | rx_calib[21]);
    int16_t  dig_P9 = (int16_t)((rx_calib[24] << 8) | rx_calib[23]);

    // 6. Continuous Data Acquisition Loop
    uint8_t tx_data[7] = {0xF7};
    uint8_t rx_data[7] = {0};
    spi_transaction_t data_trans = { .length = 56, .tx_buffer = tx_data, .rx_buffer = rx_data };

    while (1) {
        spi_device_polling_transmit(spi, &data_trans);

        // Reconstruct 20-bit raw ADC values
        int32_t adc_P = ((int32_t)rx_data[1] << 12) | ((int32_t)rx_data[2] << 4) | (rx_data[3] >> 4);
        int32_t adc_T = ((int32_t)rx_data[4] << 12) | ((int32_t)rx_data[5] << 4) | (rx_data[6] >> 4);

        // Bosch Fixed-Point Temperature Compensation
        int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
        int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
        int32_t t_fine = var1 + var2;
        float temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;

        // Bosch Fixed-Point Pressure Compensation
        int64_t p_var1, p_var2, p;
        p_var1 = ((int64_t)t_fine) - 128000;
        p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
        p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
        p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
        p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) + ((p_var1 * (int64_t)dig_P2) << 12);
        p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dig_P1) >> 33;

        if (p_var1 != 0) {
            p = 1048576 - adc_P;
            p = (((p << 31) - p_var2) * 3125) / p_var1;
            p_var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
            p_var2 = (((int64_t)dig_P8) * p) >> 19;
            p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dig_P7) << 4);
            
            float pressure = (float)p / 256.0f;
            printf("Temp: %.2f °C | Press: %.2f Pa\n", temperature, pressure);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---
