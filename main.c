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

    // 2. Device Configuration
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 0,
        .spics_io_num = 5,
        .queue_size = 7};

    // 3. Initialize SPI Host
    spi_device_handle_t spi;
    spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI3_HOST, &devcfg, &spi);

    // Wake up sensor (Write 0x27 to 0x74)
    uint8_t tx_wake[2] = {0x74, 0x27};
    spi_transaction_t wake_trans = {.length = 16, .tx_buffer = tx_wake};
    spi_device_polling_transmit(spi, &wake_trans);

    // Read 24 bytes of Calibration Data (Starting at 0x88)
    // Array size is 25: 1 byte for address, 24 bytes for data
    uint8_t tx_calib[25] = {0x88};
    uint8_t rx_calib[25] = {0};
    spi_transaction_t calib_trans = {.length = 200, .tx_buffer = tx_calib, .rx_buffer = rx_calib};
    spi_device_polling_transmit(spi, &calib_trans);

    // Reconstruct Temperature Calibration (T1 is unsigned, T2-T3 are signed)
    uint16_t dig_T1 = (rx_calib[2] << 8) | rx_calib[1];
    int16_t dig_T2 = (int16_t)((rx_calib[4] << 8) | rx_calib[3]);
    int16_t dig_T3 = (int16_t)((rx_calib[6] << 8) | rx_calib[5]);

    // Reconstruct Pressure Calibration (P1 is unsigned, P2-P9 are signed)
    uint16_t dig_P1 = (rx_calib[8] << 8) | rx_calib[7];
    int16_t dig_P2 = (int16_t)((rx_calib[10] << 8) | rx_calib[9]);
    int16_t dig_P3 = (int16_t)((rx_calib[12] << 8) | rx_calib[11]);
    int16_t dig_P4 = (int16_t)((rx_calib[14] << 8) | rx_calib[13]);
    int16_t dig_P5 = (int16_t)((rx_calib[16] << 8) | rx_calib[15]);
    int16_t dig_P6 = (int16_t)((rx_calib[18] << 8) | rx_calib[17]);
    int16_t dig_P7 = (int16_t)((rx_calib[20] << 8) | rx_calib[19]);
    int16_t dig_P8 = (int16_t)((rx_calib[22] << 8) | rx_calib[21]);
    int16_t dig_P9 = (int16_t)((rx_calib[24] << 8) | rx_calib[23]);

    // Continuous Data Acquisition Loop
    uint8_t tx_data[7] = {0xF7};
    uint8_t rx_data[7] = {0};
    spi_transaction_t data_trans = {.length = 56, .tx_buffer = tx_data, .rx_buffer = rx_data};

    while (1)
    {
        spi_device_polling_transmit(spi, &data_trans);

        // Raw ADC values (0xF7 contains Pressure MSB, 0xFA contains Temp MSB)
        int32_t adc_P = ((int32_t)rx_data[1] << 12) | ((int32_t)rx_data[2] << 4) | (rx_data[3] >> 4);
        int32_t adc_T = ((int32_t)rx_data[4] << 12) | ((int32_t)rx_data[5] << 4) | (rx_data[6] >> 4);

        // Temperature Compensation
        int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
        int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
        int32_t t_fine = var1 + var2;
        float temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;

        // Pressure Compensation (requires t_fine)
        int64_t p_var1, p_var2, p;
        p_var1 = ((int64_t)t_fine) - 128000;
        p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
        p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
        p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
        p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) + ((p_var1 * (int64_t)dig_P2) << 12);
        p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dig_P1) >> 33;

        if (p_var1 == 0)
        {
            printf("Pressure Calculation Error (Division by Zero)\n");
        }
        else
        {
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