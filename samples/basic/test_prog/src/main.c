#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>

#define SLEEP_TIME_MS   1000

#define LED0_NODE	DT_ALIAS(led0)
#define SW0_NODE	DT_ALIAS(sw0)

enum states {
    STATE_LED_OFF,
    STATE_LED_ON
};

//const struct device *led_dev;
//const struct device *button_dev;

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static struct gpio_callback button_cb;
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button pressed\n");

	int val = gpio_pin_get_dt(&button);
	static int state = STATE_LED_OFF;

	printk("state : %d\n", state);

	if (val != state)
	{
		gpio_pin_set_dt(&led, 0);
		state = STATE_LED_ON;
		printk("LED OFF\n");
	}
	else
	{
		gpio_pin_set_dt(&led, 1);
		state = STATE_LED_OFF;
		printk("LED ON\n");
	}

}
void main(void)
{
	int ret;

	if (!device_is_ready(button.port))
	{
		printk("Error: button device %s is not ready\n",
		button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n",
		ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
		ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	if (led.port && !device_is_ready(led.port))
	{
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led.port->name);
		led.port = NULL;
	}

	if (led.port)
	{
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (ret != 0)
		{
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led.port->name, led.pin);
			led.port = NULL;
		} else
		{
			printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
		}
	}
	printk("Press the button\n");
	//gpio_pin_enable_callback(button_cb, &button);

	if (led.port)
	{
		while (1)
		{
			k_msleep(SLEEP_TIME_MS);
		}
	}
}
