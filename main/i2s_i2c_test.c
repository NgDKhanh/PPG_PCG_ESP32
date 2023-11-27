#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <esp_timer.h>
#include <esp_vfs_fat.h>
#include <freertos/ringbuf.h>

// Include I2S driver
#include <driver/i2s.h>

// Incluse SD card driver
#include <sdcard.h>

// Incluse MAX30102 driver
#include "max30102.h"

// Connections to INMP441 I2S microphone
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

#define bufferCount 6
#define bufferLen 32
#define receiveBufferLen ((bufferLen * 32 / 8)  * bufferCount) 

// Buffers to store data read from dma buffers
int16_t buffer16[receiveBufferLen / 4] = {0};
uint8_t buffer32[receiveBufferLen] = {0};

// Buffer for data to save to SD card
RingbufHandle_t buf_handle_max;
RingbufHandle_t buf_handle_inm;

// Data buffer to send to ringbuffer
static char data_max[400] = "";
static char data_inm[receiveBufferLen / 4 * 6] = "";

TaskHandle_t readMAXTask_handle = NULL;
TaskHandle_t readINMTask_handle = NULL;
TaskHandle_t saveToSDTask_handle = NULL;

/**
 * @brief Read data from MAX30102 and send to ring buffer
 * 
 * @param pvParameters 
 */
void max30102_test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(max30102_initDesc(&dev, 0, 21, 22));

    struct max30102_record record;
    struct max30102_data data;

    if (max30102_readPartID(&dev) == ESP_OK) {
        ESP_LOGI(__func__, "Found MAX30102!");
    }
    else {
        ESP_LOGE(__func__, "Not found MAX30102");
    }

    if (max30102_init(0x1F, 4, 2, 1000, 118, 4096, &record, &dev) == ESP_OK) {
        ESP_LOGI(__func__, "Init OK");
    }
    else {
        ESP_LOGE(__func__, "Init fail!");
    }

    uint16_t samplesTaken = 0;
    char data_temp[16] = "";
    unsigned long red;
    unsigned long ir;
    while (1)
    {

        max30102_check(&record, &dev); //Check the sensor, read up to 3 samples

        while (max30102_available(&record)) //do we have new data?
        {
            samplesTaken++;

            // printf("%d,", max30102_getFIFORed(&record));
            // printf("%d", max30102_getFIFOIR(&record));
            // printf("\n");
            red = max30102_getFIFORed(&record);
            ir = max30102_getFIFOIR(&record);

            memset(data_temp, 0, sizeof(data_temp));
            sprintf(data_temp, "%lu,%lu\n", red, ir);
            strcat(data_max, data_temp);

            max30102_nextSample(&record); //We're finished with this sample so move to next sample
        }
        if (samplesTaken >= 25) {
            xRingbufferSend(buf_handle_max, data_max, sizeof(data_max), pdMS_TO_TICKS(5));
            samplesTaken = 0;
            memset(data_max, 0, sizeof(data_max));
        }

    }
}


// Set up I2S Processor configuration
void i2s_install() {
  // Set up I2S Processor configuration
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 4000, // or 44100 if you like
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Ground the L/R pin on the INMP441.
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S |I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = bufferCount,
    .dma_buf_len = bufferLen,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0,
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

// Set I2S pin configuration
void i2s_setpin() {
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };

    i2s_set_pin(I2S_PORT, &pin_config);
}

/**
 * @brief Read data from INMP441 and send to ring buffer
 * 
 * @param pvParameters 
 */
void readINMP441Task(void* parameter) {

    // Set up I2C
    i2s_install();
    i2s_setpin();
    i2s_start(I2S_PORT);

    size_t bytesRead = 0;
    char data_temp[8] = ""; 

    while (1)
    {
        vTaskDelay(1); // Feed for watchdog, if not watchdog timer will be triggered!
        i2s_read(I2S_PORT, &buffer32, sizeof(buffer32), &bytesRead, 100);
        int samplesRead = bytesRead / 4;

        for (uint8_t i = 0; i < samplesRead; i++) {
            uint8_t mid = buffer32[i * 4 + 2];
            uint8_t msb = buffer32[i * 4 + 3];
            uint16_t raw = (((uint32_t)msb) << 8) + ((uint32_t)mid);
            memcpy(&buffer16[i], &raw, sizeof(raw)); // Copy so sign bits aren't interfered with somehow.
            // printf("%d %d %d\n", 3000, -3000, buffer16[i]);
            
            memset(data_temp, 0, sizeof(data_temp));
            sprintf(data_temp, "\n%d", buffer16[i]);
            strcat(data_inm, data_temp);
            
        }
        
        bool res = pdFALSE;
        while (res != pdTRUE)
        {
            res = xRingbufferSend(buf_handle_inm, data_inm, sizeof(data_inm), pdMS_TO_TICKS(10));
        }
        memset(data_inm, 0, sizeof(data_inm));
    }
    
}

/**
 * @brief Receive data from 2 ring buffers and save them to SD card
 * 
 * @param parameter 
 */
void saveINMPAndMAXToSDTask(void *parameter) {
    while(1) {
        size_t item_size1;
        size_t item_size2;

        //Receive an item from no-split INMP441 ring buffer
        char *item1 = (char *)xRingbufferReceive(buf_handle_inm, &item_size1, 1);

        //Check received item
        if (item1 != NULL) {
            //Return Item
            // Serial.println("r");
            vRingbufferReturnItem(buf_handle_inm, (void *)item1);
            sdcard_writeDataToFile("pcg", item1);
        } 

        //Receive an item from no-split MAX30102 ring buffer
        char *item2 = (char *)xRingbufferReceive(buf_handle_max, &item_size2, 1);

        //Check received item
        if (item2 != NULL) {
            //Return Item
            // Serial.println("rev");
            vRingbufferReturnItem(buf_handle_max, (void *)item2);
            sdcard_writeDataToFile("ppg", item2);
        } 
    }
}

void app_main(void)
{
    // Initialize SPI Bus
    
    ESP_LOGI(__func__, "Initialize SD card with SPI interface.");
    esp_vfs_fat_mount_config_t mount_config_t = MOUNT_CONFIG_DEFAULT();
    spi_bus_config_t spi_bus_config_t = SPI_BUS_CONFIG_DEFAULT();
    sdmmc_host_t host_t = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_PIN_NUM_CS;
    slot_config.host_id = host_t.slot;

    sdmmc_card_t SDCARD;
    ESP_ERROR_CHECK(sdcard_initialize(&mount_config_t, &SDCARD, &host_t, &spi_bus_config_t, &slot_config));

    // Initialise ring buffers
    buf_handle_max = xRingbufferCreate(1028 * 6, RINGBUF_TYPE_NOSPLIT);
    buf_handle_inm = xRingbufferCreate(1028 * 15, RINGBUF_TYPE_NOSPLIT);

    if(buf_handle_inm == NULL || buf_handle_max == NULL) 
    {
        ESP_LOGE(__func__, "Ring buffers create fail");
    }
    else
    {
        ESP_LOGI(__func__, "Ring buffers create OK");
    }

    // Set up I2C
    ESP_ERROR_CHECK(i2cdev_init()); 
    
    // Create tasks
    xTaskCreatePinnedToCore(max30102_test, "max30102_test", 1024 * 5, &readMAXTask_handle, 6, NULL, 0);
    xTaskCreatePinnedToCore(readINMP441Task, "readINM411", 1024 * 15, &readINMTask_handle, 6, NULL, 0);  // ?? Make max30102 task and inm task have equal priority can make polling cycle of max3012 shorter ??
    xTaskCreatePinnedToCore(saveINMPAndMAXToSDTask, "saveToSD", 1024 * 10, &saveToSDTask_handle, 10, NULL, 1);
}
