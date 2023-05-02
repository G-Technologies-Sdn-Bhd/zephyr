#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/sensor.h>


void main(void)
{
	const struct device *dev;
	struct sensor_value val;

	dev = device_get_binding("GP2Y10");

	if (!device_is_ready(dev)) {
		printk("sensor: device not found.\n");
		return;
	}

	while (1) {

		if (sensor_sample_fetch(dev) != 0) {
			printk("sensor: sample fetch fail.\n");
		}

		if (sensor_channel_get(dev, SENSOR_CHAN_PM_10, &val) != 0) {
			printk("sensor: channel get fail.\n");
		}

		printk("sensor: dust reading: %.2f ug/m3\n", sensor_value_to_double(&val));

		k_msleep(5000);
	}

}


