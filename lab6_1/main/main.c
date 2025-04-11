
#include "driver/i2c.h"
#include <stdio.h>
#include "esp_timer.h"
#include "esp_rom_sys.h" // Add this for esp_rom_delay_us
#include "driver/gpio.h"
#include "esp_system.h"

#define I2C_MASTER_SCL_IO 8        // GPIO for SCL
#define I2C_MASTER_SDA_IO 10       // GPIO for SDA
#define I2C_MASTER_NUM I2C_NUM_0   // I2C port number
#define I2C_MASTER_FREQ_HZ 100000  // Frequency of I2C
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define SHTC3_SENSOR_ADDR 0x70     // I2C address for SHTC3
#define TRIG_PIN GPIO_NUM_5  // Trigger pin for HC-SR04
#define ECHO_PIN GPIO_NUM_4 // Echo pin for HC-SR04
#define VCC_PIN GPIO_NUM_7

// Speed of sound at 0°C in cm/µs
#define SPEED_OF_SOUND_BASE 331.3				   // 

// Updated SHTC3 command codes
#define SHTC3_POWER_UP 0x3517      // Updated power-up command
#define SHTC3_POWER_DOWN 0xB098    // Power down command
#define SHTC3_READ_TEMP_HUMID 0x7CA2 // Updated read command

// I2C Master initialization
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Power-up function with updated command
void powerUpSensor() {
    uint8_t power_up_cmd[] = { (SHTC3_POWER_UP >> 8), (SHTC3_POWER_UP & 0xFF) };
    i2c_master_write_to_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, power_up_cmd, sizeof(power_up_cmd), 1000 / portTICK_PERIOD_MS);
}

// Power-down function
void powerDownSensor() {
    uint8_t power_down_cmd[] = { (SHTC3_POWER_DOWN >> 8), (SHTC3_POWER_DOWN & 0xFF) };
    i2c_master_write_to_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, power_down_cmd, sizeof(power_down_cmd), 1000 / portTICK_PERIOD_MS);
}

// CRC-8 checksum validation
uint8_t checkCRC(uint16_t data, uint8_t crc) {
    uint8_t crc_calc = 0xFF;  
    crc_calc ^= (data >> 8);  
    for (int i = 0; i < 8; ++i) crc_calc = (crc_calc & 0x80) ? (crc_calc << 1) ^ 0x31 : (crc_calc << 1);
    crc_calc ^= (data & 0xFF);  
    for (int i = 0; i < 8; ++i) crc_calc = (crc_calc & 0x80) ? (crc_calc << 1) ^ 0x31 : (crc_calc << 1);
    return crc_calc == crc;
}

// Function to read sensor data and validate it
uint16_t readSensorData(uint16_t *humidity) {
    uint8_t read_cmd[] = { (SHTC3_READ_TEMP_HUMID >> 8), (SHTC3_READ_TEMP_HUMID & 0xFF) };
    uint8_t data[6]; // 2 bytes temp, 1 byte temp CRC, 2 bytes humidity, 1 byte humidity CRC
   
    i2c_master_write_read_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, read_cmd, sizeof(read_cmd), data, sizeof(data), 1000 / portTICK_PERIOD_MS);
    uint16_t rawTemp = (data[0] << 8) | data[1];
    uint16_t rawHumid = (data[3] << 8) | data[4];
   
    // Validate CRCs
    if (checkCRC(rawTemp, data[2]) && checkCRC(rawHumid, data[5])) {
        *humidity = rawHumid;
        return rawTemp;
    } else {
        //printf("CRC Check failed!\n");
        return 0xFFFF; // Invalid value
    }
}

// Function to calculate temperature from raw data
float calculateTemperature(uint16_t rawTemp) {
    return -45.0 + 175.0 * ((float)rawTemp / 65535.0);
}

// Function to calculate humidity from raw data
float calculateHumidity(uint16_t rawHumid) {
    return 100.0 * ((float)rawHumid / 65535.0);
}

void setupUltrasonicSensor() {
	gpio_config_t io_conf=
	{
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask =(1ULL << TRIG_PIN) | (1ULL<< VCC_PIN),
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.pull_up_en = GPIO_PULLUP_DISABLE,
	};
	gpio_config(&io_conf);

	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
	gpio_config(&io_conf);

	gpio_set_level(VCC_PIN, 1);
}

float measureDistance(float temperatureC) {
    // Adjust the speed of sound based on temperature
    float speedOfSound = SPEED_OF_SOUND_BASE + 0.606 * temperatureC; // in m/s
    speedOfSound /= 10000.0; // Convert to cm/µs

    // Trigger pulse
    gpio_set_level(TRIG_PIN, 0);
    vTaskDelay(2/portTICK_PERIOD_MS);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);  // 10 µs pulse
    gpio_set_level(TRIG_PIN, 0);

    // Measure echo start
    uint32_t start_time = esp_timer_get_time();
        while (!gpio_get_level(ECHO_PIN)) {
        if ((esp_timer_get_time() - start_time) > 50000) { // Timeout (20ms)
            printf("Timeout waiting for echo start\n");
            return -1.0;  // Invalid distance
        }
    }
    uint32_t echo_start = esp_timer_get_time();

    // Measure echo end
    while (gpio_get_level(ECHO_PIN)) {
        if ((esp_timer_get_time() - echo_start) > 50000) { // Timeout (20ms)
            printf("Timeout waiting for echo end\n");
            return -1.0;  // Invalid distance
        }  // Debounce delay
    }
    uint32_t echo_end = esp_timer_get_time();

    // Calculate distance
    float duration = (float)(echo_end - echo_start); // µs
    float distance = (duration * speedOfSound) / 2.0; // Divide by 2 for round trip

    // Debugging outputs
    printf("Echo duration: %.2f µs\n", duration);
    printf("Calculated distance: %.2f cm\n", distance);

    return distance;
}

void app_main() {
    i2c_master_init();          // Initialize I2C
    setupUltrasonicSensor();    // Initialize Ultrasonic Sensor

    while (1) {
        // Power up the sensor
        powerUpSensor();
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Small delay for power-up

        // Read temperature and humidity data
        uint16_t rawHumid;
        uint16_t rawTemp = readSensorData(&rawHumid);

        if (rawTemp != 0xFFFF) { // Only process valid data
            // Convert raw data to temperature and humidity
            float temperatureC = calculateTemperature(rawTemp);
            float humidity = calculateHumidity(rawHumid);

            // Measure distance
            float distance = measureDistance(temperatureC);

            // Display results
            if (distance >= 0) {
                printf("Distance: %.2f cm at %.1f°C\n", distance, temperatureC);
            } else {
                printf("Distance measurement failed\n");
            }
        } else {
            printf("Temperature sensor read failed\n");
        }

        // Power down the sensor
        powerDownSensor();

        // Wait for 1 second
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }    
}

