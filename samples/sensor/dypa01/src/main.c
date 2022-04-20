#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/sensor.h>


void main(void)
{
	const struct device *dev;
	struct sensor_value val;

	dev = device_get_binding("DYPA01");

	if (!device_is_ready(dev)) {
		printk("sensor: device not found.\n");
		return;
	}

	while (1) {

		if (sensor_sample_fetch(dev) != 0) {
			printk("sensor: sample fetch fail.\n");
		}

		if (sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &val) != 0) {
			printk("sensor: channel get fail.\n");
		}

		printk("sensor: distance reading: %d\n", val.val1);

		k_msleep(2000);
	}

}


