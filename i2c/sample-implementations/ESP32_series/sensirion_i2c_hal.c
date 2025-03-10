/*
 * Copyright (c) 2025, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <string.h> 

#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "sensirion_config.h"

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "rom/ets_sys.h"
#include "esp_err.h"
#include "esp_log.h"

//Redefine these macros according to board & needs
#define I2C_MASTER_NUM                     I2C_NUM_0
#define I2C_MASTER_SCL_IO                  22 
#define I2C_MASTER_SDA_IO                  21
#define I2C_MASTER_FREQ_HZ                 400000                        
#define I2C_MASTER_TIMEOUT_MS              1000

#define INFO_TAG                           "SENSIRION-HAL"
#define MAX_DEVS                           5 //Maximum amount of i2c devices

/**
 * Create a new I2C master bus instance
 */
i2c_master_bus_handle_t i2c_bus_handle;

/**
 * And an empty list of devices
 */
i2c_master_dev_handle_t device_handle_list[MAX_DEVS];

/**
 * Create a list for the different device configs. These structures shall be later accessed in order to get address for read-write functions
 */
i2c_device_config_t device_config_list[MAX_DEVS];

uint8_t plugged_devs; //Keep the amount of plugged devs to the i2c bus
/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */

void sensirion_i2c_hal_init(void) {
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        /**
         * Number of CLK cycles a signal must remain stable for it not to be ignored. 
         * If a signal is shorter than the set time, it shall be regarded as a glitch
         */
        .glitch_ignore_cnt = 7, 
        /**
         * ESP-32 i2c bus pins are open-drain. 
         * When not using an external pull up resistor, this setting allows the usage 
         * of internal weak pull up resistors on these pins.
         */
        .flags.enable_internal_pullup = true,
    };

    esp_err_t rslt;

    rslt = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle); 
    if (rslt != ESP_OK){
        ESP_LOGE(INFO_TAG, "I2C BUS INIT FAILED: %d\n", rslt);
        return; 
    }

    //Scan i2c bus to find all the plugged addresses and init them if possible

    for (uint16_t scan_addr = 1; scan_addr < 127; scan_addr++) {  
        if (i2c_master_probe(i2c_bus_handle, scan_addr, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) == ESP_OK){
            printf("Found device at 0x%X\n", scan_addr);
            if (plugged_devs > MAX_DEVS - 1){
                ESP_LOGE(INFO_TAG, "I2C INIT FAILED: FOUND TOO MANY DEVICES ON THE BUS\n");
                return;
            }
            else{
                device_config_list[plugged_devs].dev_addr_length = I2C_ADDR_BIT_LEN_7;  
                device_config_list[plugged_devs].device_address = scan_addr;
                device_config_list[plugged_devs].scl_speed_hz = I2C_MASTER_FREQ_HZ;
                rslt = i2c_master_bus_add_device(i2c_bus_handle, &device_config_list[plugged_devs], &device_handle_list[plugged_devs]);
                if (rslt != ESP_OK){
                    ESP_LOGE(INFO_TAG, "I2C DEVICE 0x%X ADD FAILED: %d\n", scan_addr, rslt);
                    return;
                }
                plugged_devs++;
            }
        }
    }
}

/**
 * Release all resources initialized by sensirion_i2c_hal_init().
 */

void sensirion_i2c_hal_free(void) {
    esp_err_t rslt;
    for(uint8_t i = 0; i < plugged_devs; i++){
        rslt = i2c_master_bus_rm_device(device_handle_list[i]);
        if (rslt != ESP_OK){
            ESP_LOGE(INFO_TAG, "I2C DEVICE 0x%X REMOVE FAILED: %d\n", device_config_list[i].device_address, rslt);
            return;
        }
    }

    plugged_devs = 0;
    memset(device_config_list, 0, sizeof(device_config_list));
    memset(device_handle_list, 0, sizeof(device_handle_list));
    
    if (i2c_bus_handle == NULL){
        ESP_LOGE(INFO_TAG, "I2C FREE FAILED: %s\n", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    (void) i2c_del_master_bus(i2c_bus_handle);
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint16_t count) {
    if (address == 0 || data == NULL || count == 0) {
        ESP_LOGE(INFO_TAG, "I2C READ FAILED: %s\n", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return 1;  
    }

    esp_err_t rslt;
    uint8_t i;
    for(i = 0; i < plugged_devs; i++){
        if(device_config_list[i].device_address == address){
            break;
        }
    }
    rslt = i2c_master_receive(device_handle_list[i], data, (size_t) count, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return (int8_t) rslt;   
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint16_t count) {
    if (address == 0 || data == NULL || count == 0) {
        ESP_LOGE(INFO_TAG, "I2C WRITE FAILED: %s\n", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return 1; 
    }

    esp_err_t rslt;
    uint8_t i;
    for(i = 0; i < plugged_devs; i++){
        if(device_config_list[i].device_address == address){
            break;
        }
    }
    rslt = i2c_master_transmit(device_handle_list[i], data, (size_t) count, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return (int8_t) rslt;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    ets_delay_us(useconds);
}
