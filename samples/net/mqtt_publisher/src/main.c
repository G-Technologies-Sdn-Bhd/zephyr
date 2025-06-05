#include <zephyr/kernel.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ppp.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define MQTT_BROKER_ADDR "test.mosquitto.org"
#define MQTT_BROKER_PORT 1883
#define CLIENTID "zephyr_modem_client"

static struct mqtt_client client;
static struct sockaddr_storage broker;
static uint8_t rx_buffer[512], tx_buffer[512], payload_buf[128];

static int resolve_broker(void)
{
	struct addrinfo *res;
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM
	};
	int err = getaddrinfo(MQTT_BROKER_ADDR, NULL, &hints, &res);
	if (err) {
		LOG_ERR("DNS resolve failed: %d", err);
		return err;
	}

	struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;
	broker4->sin_family = AF_INET;
	broker4->sin_port = htons(MQTT_BROKER_PORT);
	broker4->sin_addr.s_addr = ((struct sockaddr_in *)res->ai_addr)->sin_addr.s_addr;

	freeaddrinfo(res);
	return 0;
}

static void mqtt_evt_handler(struct mqtt_client *const c, const struct mqtt_evt *evt)
{
	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		LOG_INF("MQTT connected");
		break;
	case MQTT_EVT_DISCONNECT:
		LOG_INF("MQTT disconnected");
		break;
	default:
		LOG_INF("MQTT event %d", evt->type);
		break;
	}
}

static int mqtt_setup_and_connect(void)
{
	mqtt_client_init(&client);

	client.broker = &broker;
	client.evt_cb = mqtt_evt_handler;
	client.client_id.utf8 = (uint8_t *)CLIENTID;
	client.client_id.size = strlen(CLIENTID);
	client.protocol_version = MQTT_VERSION_3_1_1;

	client.rx_buf = rx_buffer;
	client.rx_buf_size = sizeof(rx_buffer);
	client.tx_buf = tx_buffer;
	client.tx_buf_size = sizeof(tx_buffer);

	struct mqtt_utf8 username = {.utf8 = NULL, .size = 0};
	struct mqtt_utf8 password = {.utf8 = NULL, .size = 0};
	client.user_name = &username;
	client.password = &password;

	client.transport.type = MQTT_TRANSPORT_NON_SECURE;

	int err = mqtt_connect(&client);
	if (err) {
		LOG_ERR("mqtt_connect failed: %d", err);
		return err;
	}

	return 0;
}

void main(void)
{
	LOG_INF("Zephyr MQTT modem sample");

	/* Ensure modem/PPP is up */
	struct net_if *iface = net_if_get_default();
	// net_dhcpv4_stop(iface); // Just to be safe if DHCP is enabled

	LOG_INF("Waiting for network...");
	while (!net_if_is_up(iface)) {
		k_sleep(K_SECONDS(1));
	}

	LOG_INF("Network is up");

	if (resolve_broker() != 0) {
		LOG_ERR("Broker DNS resolution failed");
		return;
	}

	if (mqtt_setup_and_connect() != 0) {
		LOG_ERR("MQTT connection failed");
		return;
	}

	while (1) {
		mqtt_input(&client);
		mqtt_live(&client);
		k_sleep(K_SECONDS(1));
	}
}
