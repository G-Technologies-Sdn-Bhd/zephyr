#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <string.h>
#include <sys/printk.h>

#define I2C_DEV_NAME DT_INST_0_SENSIRION_SHT3XD_LABEL
#define I2C_ADDRESS DT_INST_0_SENSIRION_SHT3XD_BASE_ADDRESS
#define SCL_PIN DT_INST_0_SENSIRION_SHT3XD_SCL_GPIOS_PIN
#define SDA_PIN DT_INST_0_SENSIRION_SHT3XD_SDA_GPIOS_PIN
#define TEMPERATURE_REG 0xE3
#define HUMIDITY_REG 0xE5

void main(void)
{
    struct device *i2c_dev;
    uint8_t buffer[6];

    i2c_dev = device_get_binding(I2C_DEV_NAME);
    if (!i2c_dev) {
        printk("Error: Device not found.\n");
        return;
    }

    while (1) {
        if (i2c_read(i2c_dev, buffer, sizeof(buffer), I2C_ADDRESS) == 0) {
            uint16_t temperature = (buffer[0] << 8) | buffer[1];
            uint16_t humidity = (buffer[3] << 8) | buffer[4];
            temperature *= 175;
            temperature /= 65535;
            temperature -= 45;
            humidity *= 100;
            humidity /= 65535;
            printk("Temperature: %d C\n", temperature);
            printk("Humidity: %d%%\n", humidity);
        } else {
            printk("Error: Failed to read from the sensor.\n");
        }

        k_sleep(K_SECONDS(1));
    }
}
