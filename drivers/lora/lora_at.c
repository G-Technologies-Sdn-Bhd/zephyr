/*
 * Copyright (c) 2021-2022 G-Technologies Sdn. Bhd. - All Rights Reserved
 *
 * Unauthorized copying and distributing of this file, via any medium is
 * strictly prohibited.
 *
 * If received in error, please contact G-Technologies Sdn. Bhd. at
 * info@gtsb.com.my, quoting the name of the sender and the addressee, then
 * delete it from your system.
 *
 * Proprietary and confidential
 */


#define DT_DRV_COMPAT ebyte_e78
#include <logging/log.h>
#include <zephyr.h>
#include <drivers/lora.h>
LOG_MODULE_REGISTER(loraat, CONFIG_LORA_LOG_LEVEL);

#include <stdlib.h>
#include <kernel.h>
#include <device.h>
#include <sys/ring_buffer.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <sys/reboot.h>
#include <net/ppp.h>
#include <drivers/modem/gsm_ppp.h>
#include <drivers/modem/quectel.h>
#include <drivers/uart.h>
#include <drivers/console/uart_mux.h>
#include"drivers/lora_at.h"
#include "gsm_ppp.h"
#include "modem_context.h"
#include "modem_iface_uart.h"
#include "modem_cmd_handler.h"
#include "../console/gsm_mux.h"
#include <stdio.h>
// #include <pm/pm.h>
#include <time.h>
#include <sys/byteorder.h>
#include "lib/do_ctrl.h"
#include "components/seismic.h"

#define LORA_AT_UART_NODE                   DT_INST_BUS(0)
#define LORA_AT_CMD_READ_BUF                128
#define LORA_AT_CMD_AT_TIMEOUT              K_SECONDS(10)
#define LORA_AT_CMD_SETUP_TIMEOUT           K_SECONDS(10)
#define LORA_AT_RX_STACK_SIZE               1024//512//CONFIG_MODEM_LORA_AT_RX_STACK_SIZE
#define LORA_AT_WORKQ_STACK_SIZE           768//CONFIG_MODEM_LORA_AT_WORKQ_STACK_SIZE
#define LORA_AT_RECV_MAX_BUF                30
#define LORA_AT_RECV_BUF_SIZE               128
#define LORA_AT_ATTACH_RETRY_DELAY_MSEC     1000
#define LORA_AT_REGISTER_DELAY_MSEC         1000
#define LORA_AT_REGISTER_TIMEOUT            30//60
#define CONFIRM 1
#define NBTRIALS 2
#define A_PPP_MRU 1500
#define APPKEY CONFIG_LORA_AT_APPKEY
#define APPEUI CONFIG_LORA_AT_APPEUI
enum lora_data_flag{
	LORA_CONNECTING = BIT(1),
	LORA_CONNECTED = BIT(2),
};

static struct lora_modem{
	const struct device *dev;
	struct net_if *net_iface;

	struct modem_context context;

	struct k_mutex lock;
	struct k_work_delayable status_dwork;
	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_match_buf[LORA_AT_CMD_READ_BUF];
	struct k_sem sem_response;
	// uint64_t recv_data;
	struct modem_iface_uart_data lora_at_data;
	struct k_work_delayable lora_at_configure_work;
	char lora_at_rx_rb_buf[A_PPP_MRU * 3];
	int8_t *ppp_recv_buf;
	size_t ppp_recv_buf_len;

	const struct device *ppp_dev;
	const struct device *at_dev;
	const struct device *control_dev;

	struct k_thread rx_thread;
	bool workq_plug;
	struct k_work_q workq;
	struct k_work_delayable rssi_work_handle;
	struct gsm_ppp_modem_info minfo;
	bool conn_status;
	uint8_t flags;
	enum lora_state{
		LORA_START,
		LORA_STOP,
		LORA_STATE_ERROR,
	}state;
	// struct net_if *iface;
	lora_cb lora_status;
	lora_connect_cb connect;
	uint8_t recv_data;
	uint8_t lora_con_status;
}lora;


NET_BUF_POOL_DEFINE(lora_at_recv_pool, LORA_AT_RECV_MAX_BUF, LORA_AT_RECV_BUF_SIZE,
		    0, NULL);
K_KERNEL_STACK_DEFINE(lora_at_rx_stack, LORA_AT_RX_STACK_SIZE);
K_KERNEL_STACK_DEFINE(lora_at_workq_stack, LORA_AT_WORKQ_STACK_SIZE);

void lorawan_register_modem_status_callback(lora_cb cb)
{
	// struct lora_modem *lora =dev->data;
	lora.lora_status =cb;
}
void lorawan_status_callback(lora_connect_cb cb)
{
		lora.connect = cb;
}
int LORA_MGMT_RAISE_CONNECT_RESULT_EVENT(void)
{
	if(lora.conn_status == true)
	{
		return 0;
	}
	return -1;
} 
static inline bool lora_flag_set(struct lora_modem *dev,uint8_t flags)
{
	dev->flags |= flags;
}

static inline void lora_flags_clear(struct lora_modem *dev, uint8_t flags)
{
	dev->flags &= (~flags);
}

static inline bool lora_flags_are_set(struct lora_modem *dev, uint8_t flags)
{
	return (dev->flags & flags) != 0;
}


static inline void lora_at_lock(struct lora_modem *lora)
{
	int ret = k_mutex_lock(&lora->lock, K_FOREVER);

	__ASSERT(ret == 0, "%s failed: %d", "k_mutex_lock", ret);
}

static inline void lora_at_unlock(struct lora_modem *lora)
{
	int ret = k_mutex_unlock(&lora->lock);

	__ASSERT(ret == 0, "%s failed: %d", "k_mutex_unlock", ret);
}

static void lora_at_rx(struct lora_modem *lora)
{
	LOG_DBG("starting");
	
	while (true) {
		(void)k_sem_take(&lora->lora_at_data.rx_sem, K_FOREVER);
		/* The handler will listen AT channel */
		lora->context.cmd_handler.process(&lora->context.cmd_handler,
						 &lora->context.iface);
		k_yield();
	}
}
// static int lora_atoi(const char *s, const int err_value,
// 				const char *desc, const char *func)
// {
// 	int ret;
// 	char *endptr;

// 	ret = (int)strtol(s, &endptr, 10);
// 	if (!endptr || *endptr != '\0') {
// 		LOG_ERR("bad %s '%s' in %s", log_strdup(s),
// 			 log_strdup(desc), log_strdup(func));
// 		return err_value;
// 	}

// 	return ret;
// }
int lora_atoi_array(const char *s, int *buff, int array_size)
{
//  char input_string[] = ":02,00,00";
char *token;

// Use the strtok function to split the string by ","
token = strtok(s, ",");
int i =0;
// Loop through and print the values
while (token != NULL) {
    // Assuming you want to convert the values to integers
    buff[i]= atoi(token);
	i++;
    // Do something with the value, e.g., print it   
    // Get the next token
    token = strtok(NULL, ",");
}
}



void convertStringToInt(const char *dataString, int *data) {
    // Check for NULL pointer or empty string
    if (dataString == NULL || dataString[0] == '\0') {
        *data = 0; // Default to 0 in case of invalid input
        return;
    }

    char *endptr;
    *data = strtol(dataString, &endptr, 10);

    // Check if conversion was successful
    if (*endptr != '\0') {
        // Handle conversion error, e.g., invalid characters in the input string
        // You may add appropriate error handling based on your use case.
        *data = 0; // Default to 0 in case of invalid input
    }
}
int lora_at_get_command(void){
	int d = lora.recv_data;
	if(lora.recv_data >0)
	{
		lora.recv_data = 0;
	}
	return d;

}
// 02,00,00
MODEM_CMD_DEFINE(lora_cmd_rev)
{
	if(atoi(argv[2])>0)
	{
		LOG_WRN("GOT Data %d",atoi(argv[3]));
		lora.recv_data = atoi(argv[3]);
		lora_set_origin_axis();
		

	}
	modem_cmd_handler_set_error(data, 0);
	LOG_INF("%s","RECV: OK");
	k_sem_give(&lora.sem_response);
	return 0;
}
MODEM_CMD_DEFINE(lora_cmd_ok)
{
	LOG_DBG("OK");
	// MODEM_CMD("+RECV", lora_cmd_rev, 0U, "")
	size_t out_len;
	char md[7];
	char md2[7];
	out_len = net_buf_linearize(md,
				    sizeof(md) -1,
				    data->rx_buf, 0, len);
	md[out_len+1] = '\0';

	LOG_INF("MD:%s", log_strdup(md));
	
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&lora.sem_response);
	return 0;
}
MODEM_CMD_DEFINE(on_cmd_status)
{
	modem_cmd_handler_set_error(data, 0);
	size_t out_len;
	char status[20];
	int ret;
	
	out_len = net_buf_linearize(status,
				    sizeof(status) -1,
				    data->rx_buf,0,len);

	lora.lora_con_status = atoi(status);
	status[out_len+1] = '\0';	
	k_sem_give(&lora.sem_response);
	return 0;	
}
MODEM_CMD_DEFINE(lora_cmd_error)
{
	modem_cmd_handler_set_error(data, -EINVAL);
	size_t out_len;
	char md[20];
	char md2[7];
	out_len = net_buf_linearize(md,
				    sizeof(md) -1,
				    data->rx_buf, 0, len);
	md[out_len+1] = '\0';

	LOG_ERR("error:%s", log_strdup(md));
	// LOG_ERR("ERR");
	k_sem_give(&lora.sem_response);
	return 0;
}

/*
 * Provide modem info if modem shell is enabled. This can be shown with
 * "modem list" shell command.
 */

/* Handler: <manufacturer> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_manufacturer)
{
	size_t out_len;

	out_len = net_buf_linearize(lora.minfo.mdm_manufacturer,
				    sizeof(lora.minfo.mdm_manufacturer) -1,
				    data->rx_buf, 0, len);
	lora.minfo.mdm_manufacturer[out_len+1] = '\0';
	LOG_INF("Manufacturer: %s", log_strdup(lora.minfo.mdm_manufacturer));
k_sem_give(&lora.sem_response);
	return 0;
}

/* Handler: <model> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_model)
{
	size_t out_len;

	out_len = net_buf_linearize(lora.minfo.mdm_model,
				    sizeof(lora.minfo.mdm_model) - 1,
				    data->rx_buf, 0, len);
	lora.minfo.mdm_model[out_len] = '\0';
	LOG_INF("Model: %s", log_strdup(lora.minfo.mdm_model));
	k_sem_give(&lora.sem_response);
	return 0;
}
/* Handler: <rev> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_revision)
{
	size_t out_len;

	out_len = net_buf_linearize(lora.minfo.mdm_revision,
				    sizeof(lora.minfo.mdm_revision) - 1,
				    data->rx_buf, 0, len);
	lora.minfo.mdm_revision[out_len] = '\0';
	LOG_INF("Revision: %s", log_strdup(lora.minfo.mdm_revision));
	k_sem_give(&lora.sem_response);
	return 0;
}

/* Handler: <IMEI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imei)
{
	size_t out_len;

	out_len = net_buf_linearize(lora.minfo.mdm_imei, sizeof(lora.minfo.mdm_imei) - 1,
				    data->rx_buf, 0, len);
	lora.minfo.mdm_imei[out_len] = '\0';
	LOG_INF("IMEI: %s", log_strdup(lora.minfo.mdm_imei));
	k_sem_give(&lora.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_setup_no_handler)
{
	LOG_INF("on_cmd_setup_no_handler");
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&lora.sem_response);
	return 0;
}


static const struct setup_cmd setup_modem_info_cmds[] ={
		/* query modem info */
	SETUP_CMD("AT+CGMI?", "+CGMI=", on_cmd_atcmdinfo_manufacturer, 0U, ""),
	SETUP_CMD("AT+CGMM?", "+CGMM=", on_cmd_atcmdinfo_model, 0U, ""),
	SETUP_CMD("AT+CGMR?", "+CGMR=", on_cmd_atcmdinfo_revision, 0U, ""),
	SETUP_CMD("AT+CGSN?", "+CGSN=", on_cmd_atcmdinfo_imei, 0U, ""),
	SETUP_CMD("AT+CSTATUS?", "+CSTATUS:", on_cmd_status, 0U, ""),
	// SETUP_CMD("AT+CJOIN=1,0,8,8","OK",on_cmd_setup_no_handler,0U,""),
};

static int lora_query_modem_info(struct lora_modem *lora)
{
	int ret;

	ret =  modem_cmd_handler_setup_cmds(&lora->context.iface,
					    &lora->context.cmd_handler,
					    setup_modem_info_cmds,
					    ARRAY_SIZE(setup_modem_info_cmds),
						&lora->sem_response,
					    LORA_AT_CMD_SETUP_TIMEOUT);

	if (ret < 0) {
		return ret;
	}

	return 0;
}
MODEM_CMD_DEFINE(on_cmd_atcmdjoin)
{
	struct lora_modem *dev = CONTAINER_OF(data,struct lora_modem,
						cmd_handler_data);

	size_t out_len;
	char status[20];
	int ret;
	
	out_len = net_buf_linearize(status,
				    sizeof(status) -1,
				    data->rx_buf,0,len);

	status[out_len] = '\0';


	modem_cmd_handler_set_error(data, 0);
	
		if(lora_flags_are_set(dev,LORA_CONNECTED)){
			return 0;
		}
	TURN_DO_(green_led,_ON);
	int d =lora_flag_set(dev,LORA_CONNECTED);
	if (strcmp(status, "OK") == 0){
		LOG_INF("LORA CONNECTED %s",log_strdup(status));
	}
	else{
		lora_rejoin();	
		LOG_ERR("LORA FAILED %s",log_strdup(status));

	}
	lora.connect(4);
	lora.conn_status = true;

	k_sem_give(&lora.sem_response);
	return 0;
}
static const struct modem_cmd response_cmds[] = {
	MODEM_CMD_ARGS_MAX("OK+RECV:", lora_cmd_rev, 3U,4U,","),
	// MODEM_CMD("OK", lora_cmd_ok, 0U,""),
	// MODEM_CMD("OK+SEND", lora_cmd_dtrx, 0U,":"),
	MODEM_CMD("+CJOIN:", on_cmd_atcmdjoin, 0U, "OK"),
	MODEM_CMD("ERR", lora_cmd_error, 0U, ""),
	// MODEM_CMD("+CSTATUS",on_cmd_status,0U,":"),
};
void lora_rejoin(void)
{
	int ret;

	ret =modem_cmd_send(&lora.context.iface,
			&lora.context.cmd_handler,
			NULL, 0U,
			"AT+CJOIN=1,1,8,8",
			&lora.sem_response,
			K_SECONDS(2));

}
static const struct setup_cmd  status_get_cmd[]={
	SETUP_CMD("AT+CSTATUS?", "+CSTATUS:", on_cmd_status, 0U, ""),
};

void lora_work(struct k_work *work)
{
	int ret;

	LOG_WRN("Lora status work %d",lora.lora_con_status);
	if(lora.conn_status == true)
	{
		return ;
	}
	
	if(lora.lora_con_status ==0)
	{
// lora_query_modem_info(&lora);
		ret =  modem_cmd_handler_setup_cmds(&lora.context.iface,
					    &lora.context.cmd_handler,
					    status_get_cmd,
					    ARRAY_SIZE(status_get_cmd),
						&lora.sem_response,
					    LORA_AT_CMD_SETUP_TIMEOUT);
	k_work_reschedule(&lora.status_dwork,K_SECONDS(8));
	}
	else
	{
		// if(lora.lora_con_status ==)	
		// if(lora.conn_status == false)
		// {
			TURN_DO_(green_led,_ON);
			lora.connect(lora.lora_con_status);
			lora.conn_status = true;
		// }
	}
	// if()
}

static const struct setup_cmd setup_modem_join[] ={
	SETUP_CMD("AT+CCLASS=0","OK",on_cmd_setup_no_handler,0U,""),
	SETUP_CMD("AT+CJOINMODE=0","OK",on_cmd_setup_no_handler,0U,""),
	SETUP_CMD("AT+CJOIN=1,0,8,8","OK",on_cmd_setup_no_handler,0U,""),
};


int lora_at_config(const struct device *dev,char *data);
static int lora_at_init(const struct device *dev)
{
	struct lora_modem *lora = dev->data;
	int r;	
	lora->dev = dev;
	TURN_DO_(blue_led,_OFF);
	(void)k_mutex_init(&lora->lock);

	lora->cmd_handler_data.cmds[CMD_RESP] = response_cmds;
	lora->cmd_handler_data.cmds_len[CMD_RESP] = ARRAY_SIZE(response_cmds);
	// gsm->cmd_handler_data.cmds[CMD_UNSOL] = unsol_cmds;
	// lora->cmd_handler_data.cmds_len[CMD_UNSOL] = ARRAY_SIZE(unsol_cmds);
	lora->cmd_handler_data.match_buf = &lora->cmd_match_buf[0];
	lora->cmd_handler_data.match_buf_len = sizeof(lora->cmd_match_buf);
	lora->cmd_handler_data.buf_pool = &lora_at_recv_pool;
	lora->cmd_handler_data.alloc_timeout = K_NO_WAIT;
	lora->cmd_handler_data.eol = "\r";
	lora->cmd_handler_data.use_mutex = true;

	k_sem_init(&lora->sem_response, 0, 1);

	r = modem_cmd_handler_init(&lora->context.cmd_handler,
				   &lora->cmd_handler_data);
				 
	if (r < 0) {
		// LOG_DBG("cmd handler error %d", r);
		return r;
	}
 
	/* modem information storage */
	lora->context.data_manufacturer = lora->minfo.mdm_manufacturer;
	lora->context.data_model = lora->minfo.mdm_model;
	lora->context.data_revision = lora->minfo.mdm_revision;
	lora->context.data_imei = lora->minfo.mdm_imei;

	lora->lora_at_data.hw_flow_control = DT_PROP(LORA_AT_UART_NODE, hw_flow_control);
	lora->lora_at_data.rx_rb_buf = &lora->lora_at_rx_rb_buf[0];
	lora->lora_at_data.rx_rb_buf_len = sizeof(lora->lora_at_rx_rb_buf);


	r = modem_iface_uart_init(&lora->context.iface, &lora->lora_at_data,
				DEVICE_DT_GET(LORA_AT_UART_NODE));
	if (r < 0) {
		LOG_DBG("iface uart error %d", r);
		return r;
	}

	r = modem_context_register(&lora->context);
	if (r < 0) {
		LOG_DBG("context error %d", r);
		return r;
	}

	k_thread_create(&lora->rx_thread, lora_at_rx_stack,
			K_KERNEL_STACK_SIZEOF(lora_at_rx_stack),
			(k_thread_entry_t) lora_at_rx,
			lora, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_name_set(&lora->rx_thread, "lora_at_rx");

	/* initialize the work queue */
	k_work_queue_init(&lora->workq);
	k_work_queue_start(&lora->workq, lora_at_workq_stack, K_KERNEL_STACK_SIZEOF(lora_at_workq_stack),
			   K_PRIO_COOP(7), NULL);
	k_thread_name_set(&lora->workq.thread, "lora_at_workq");
	k_work_init_delayable(&lora->status_dwork,lora_work);
// const struct device *uart_dev = DEVICE_DT_GET(LORA_AT_UART_NODE);

	// 	uart_irq_rx_disable(LORA_AT_UART_NODE);
	// uart_irq_tx_disable(LORA_AT_UART_NODE);

	// uart_flush(LORA_AT_UART_NODE);
// uart_irq_rx_enable(uart_dev);
// 	uart_irq_tx_enable(uart_dev);
	lora->conn_status = false;
	// lora_at_lock(lora);
	lora->state =LORA_STOP;
	lora->lora_con_status =0;
// lora_at_config(dev,NULL);
	// lora_rejoin();
	// lora_query_modem_info(lora);	
	// lora_at_unlock(lora);
	// net_if_up(lora->net_iface);
	// if(r!=0){
	// 	sys_reboot(0);
	// }
	// k_msleep(1000);
	return 0;
	
}

int lora_at_config(const struct device *dev,char *data)
{
	int ret;

	// char *d ="199000002";
	//  0000000001100001
	char cdeveui[sizeof("AT+CDEVEUI=\"#################\"")] = {0};
	char appeuid[sizeof("AT+CAPPEUI=\"#################\"")] = {0};
	snprintk(cdeveui, sizeof(cdeveui), "AT+CDEVEUI=0000000%s",data);
	// snprintk(appeuid, sizeof(appeuid), "AT+CAPPEUI=0000000%d",(int)data);
	snprintk(appeuid, sizeof(appeuid), "AT+CAPPEUI=0000000%s",data);
	// LOG_INF("%s",log_strdup(cdeveui));		   
	ret = modem_cmd_send(&lora.context.iface,
				&lora.context.cmd_handler,
				NULL, 0U,
				// "AT+CAPPKEY=CA1F35221765D6C4499EDBCADA383764",
				"AT+CAPPKEY="APPKEY,				
				&lora.sem_response,
				LORA_AT_CMD_SETUP_TIMEOUT);
	ret = modem_cmd_send(&lora.context.iface,
				&lora.context.cmd_handler,
				NULL, 0U,
				// "AT+CAPPEUI=0000000",
				//  "AT+CAPPEUI=0000000%s"data,
				appeuid,
				&lora.sem_response,
				LORA_AT_CMD_SETUP_TIMEOUT);
	ret = modem_cmd_send(&lora.context.iface,
			&lora.context.cmd_handler,
			NULL, 0U,
			// "AT+CDEVEUI=1000000000000001",//1000000000000011",
			cdeveui,
			&lora.sem_response,
			LORA_AT_CMD_SETUP_TIMEOUT);
	ret = modem_cmd_send(&lora.context.iface,
			&lora.context.cmd_handler,
			NULL, 0U,
			"AT+CFREQBANDMASK=0002",
			&lora.sem_response,
			LORA_AT_CMD_SETUP_TIMEOUT);

	ret =  modem_cmd_handler_setup_cmds(&lora.context.iface,
					&lora.context.cmd_handler,
					setup_modem_join,
					ARRAY_SIZE(setup_modem_join),
					&lora.sem_response,
					LORA_AT_CMD_SETUP_TIMEOUT);
	if(lora.conn_status != true)
	{
			lora.conn_status= false;
	}				
	LOG_INF("cdeveui: %s\r\n appeuid:%s",log_strdup(cdeveui),log_strdup(appeuid));
	return ret;
}

// static const struct setup_cmd setup_modem_join[] ={
// 	SETUP_CMD_NOHANDLE("AT+DTRX="), 1,2,3,112233
// 	// SETUP_CMD("AT+CJOIN=1,0,8,8","",on_cmd_atcmdjoin,3U,","),
// };

MODEM_CMD_DEFINE(on_cmd_sock_readdata)
{
	LOG_INF("ssssssssssssssssssssssss");
}
int lora_at_send(const struct device *dev, char *data,
		     uint32_t data_len)
{
	
	int  ret;
	struct lora_modem *d = dev->data;
	
	struct modem_cmd cmd  = MODEM_CMD("OK+SEND", on_cmd_setup_no_handler, 0U, "");
	
	if(lora.conn_status == true)
	{
	lora_at_lock(d);
		char buf[data_len +19];
		snprintk(buf, sizeof(buf), "AT+DTRX=%d,%d,%d,%s", CONFIRM,NBTRIALS,data_len,data);
		ret = modem_cmd_send(&lora.context.iface,
					&lora.context.cmd_handler,
					&response_cmds[0],0U,
					buf,
					&lora.sem_response,
					LORA_AT_CMD_SETUP_TIMEOUT);
		// if(ret<0)
		// {
		// 	sys_reboot(SYS_REBOOT_COLD);
		// }
	return ret;
	lora_at_unlock(d);
	}else{
		LOG_INF("No connection");
		return -1;
	}
	
}


static const struct lora_driver_api lora_at_api = {
	.config = lora_at_config,
	.send = lora_at_send,
	// .send_async = lora_at_send_async,
	// .recv = lora_at_recv,
	// .test_cw = lora_at_test_cw,
};
static bool lora_info =false;
void lora_start(const struct device *dev)
{
	struct lora_modem *lora = dev->data;
	lora_at_lock(lora);

	if(lora->state !=LORA_STOP)
	{
		LOG_ERR("lora_at is already %s", "started");
		goto unlock;
	}

	lora->state = LORA_START;
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lora_pwr_en), okay)
		TURN_DO_(lora_pwr_en, _ON);
		k_msleep(2000);
 #endif
	
	if(lora_info == false)
	{
	// lora_at_config(lora,NULL);	
	lora_query_modem_info(lora);
	lora_rejoin();
	k_work_reschedule(&lora->status_dwork,K_SECONDS(8));
	}

	lora_info= true;

	lora->lora_status(LORA_EVT_STATED);

unlock:
lora_at_unlock(lora);
}

void lora_stop(const struct device *dev)
{
	struct lora_modem *lora = dev->data;
	// lora_at_lock(lora);
	if(lora->state ==LORA_STOP)
	{
		LOG_ERR("lora_at is already %s", "stop");
		return;
	}
	if (lora->conn_status!=true)
	{
		LOG_ERR("lora_at is still  %s", "not connected");
		return;
	}
lora->lora_status(LORA_EVT_STOP);
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lora_pwr_en), okay)
	TURN_DO_(lora_pwr_en, _OFF);
#endif
	lora->state = LORA_STOP;

}


DEVICE_DT_DEFINE(DT_INST(0, ebyte_e78), lora_at_init, NULL, &lora, NULL,
		 POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,&lora_at_api);


