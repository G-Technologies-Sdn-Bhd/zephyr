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

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ppp.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/lora_at.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/modem/gsm_ppp.h>
#include <zephyr/drivers/modem/quectel.h>
#include <zephyr/drivers/console/uart_mux.h>
#include <zephyr/drivers/modem/gsm_ppp.h>
// #include "lib/do_ctrl.h"
#include "modem_context.h"
#include "modem_iface_uart.h"
#include "modem_cmd_handler.h"
#include "../console/gsm_mux.h"
// #include "components/seismic.h"

// #include <pm/pm.h>

LOG_MODULE_REGISTER(loraat, CONFIG_LORA_LOG_LEVEL);

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
		// lora_set_origin_axis();


	}
	modem_cmd_handler_set_error(data, 0);
	LOG_INF("%s : [%d| %d| %d]","RECV: OK", atoi(argv[1]),atoi(argv[2]),atoi(argv[3]));
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

	LOG_INF("MD:%s", md);

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
	if( atoi(status)==4 && lora.conn_status== false){
		lora.lora_con_status = atoi(status);
	}
	status[out_len+1] = '\0';
	k_sem_give(&lora.sem_response);
	return 0;
}
MODEM_CMD_DEFINE(lora_cmd_error)
{
	modem_cmd_handler_set_error(data, -EINVAL);
	size_t out_len;
	static 	uint8_t count_try=0;
	char md[20];
	char md2[7];
	out_len = net_buf_linearize(md,
				    sizeof(md) -1,
				    data->rx_buf, 0, len);
	md[out_len+1] = '\0';

	LOG_ERR("error:%s ,%d", md,atoi(md));
	// LOG_ERR("ERR");
	switch(atoi(md))
	{
		case 0:
		LOG_INF("NOT CONNECTED");
		// lora_rejoin();

		break;
		default:
		LOG_ERR("Unkown State %d",atoi(md));
		break;
	}
	// if(count_try++>5)
	// {
		// sys_reboot(0);
	// }
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
	LOG_INF("Manufacturer: %s", lora.minfo.mdm_manufacturer);
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
	LOG_INF("Model: %s", lora.minfo.mdm_model);
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
	LOG_INF("Revision: %s", lora.minfo.mdm_revision);
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
	LOG_INF("IMEI: %s", lora.minfo.mdm_imei);
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
			ret =0;
		}
	//TURN_DO_(green_led,_ON);
	if (strcmp(status, "OK") == 0){
	int d =lora_flag_set(dev,LORA_CONNECTED);

		LOG_INF("LORA CONNECTED %s",status);
		// lora.lora_status(2);
		lora.conn_status = true;
		// lora.connect(4);
		ret =0;
	}
	else{
		// lora_rejoin();
		LOG_ERR("LORA FAILED %s",status);

	ret= -ENOTCONN;
	}
	k_sem_give(&lora.sem_response);
	return ret;
}
static const struct modem_cmd response_cmds[] = {
	MODEM_CMD_ARGS_MAX("OK+RECV:", lora_cmd_rev, 3U,4U,","),
	MODEM_CMD("+CJOIN:", on_cmd_atcmdjoin, 0U, ""),
	// MODEM_CMD("ERR", lora_cmd_error, 0U, ""),
	MODEM_CMD("ERR", lora_cmd_error, 0U, ":"),
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
			K_SECONDS(20));

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
			//TURN_DO_(green_led,_ON);
			lora.connect(lora.lora_con_status);
			lora.conn_status = true;
	}
}

static const struct setup_cmd setup_modem_join[] ={
	SETUP_CMD("AT+CCLASS=0","OK",on_cmd_setup_no_handler,0U,""),
	SETUP_CMD("AT+CJOINMODE=0","OK",on_cmd_setup_no_handler,0U,""),
	SETUP_CMD("AT+CJOIN=1,1,8,8","OK",on_cmd_setup_no_handler,0U,""),
};


int lora_at_config(const struct device *dev,char *data);
static int lora_at_init(const struct device *dev)
{
	struct lora_modem *lora = dev->data;
	int r;
	lora->dev = dev;
	//TURN_DO_(blue_led,_OFF);
	(void)k_mutex_init(&lora->lock);

	k_sem_init(&lora->sem_response, 0, 1);

	/* cmd handler setup */
	const struct modem_cmd_handler_config cmd_handler_config = {
		.match_buf = &lora->cmd_match_buf[0],
		.match_buf_len = sizeof(lora->cmd_match_buf),
		.buf_pool = &lora_at_recv_pool,
		.alloc_timeout = K_NO_WAIT,
		.eol = "\r",
		.user_data = NULL,
		.response_cmds = response_cmds,
		.response_cmds_len = ARRAY_SIZE(response_cmds),
		// .unsol_cmds = unsol_cmds,
		// .unsol_cmds_len = ARRAY_SIZE(unsol_cmds),
	};

	r = modem_cmd_handler_init(&lora->context.cmd_handler,
				   &lora->cmd_handler_data, &cmd_handler_config);

	if (r < 0) {
		return r;
	}

	/* modem information storage */
	lora->context.data_manufacturer = lora->minfo.mdm_manufacturer;
	lora->context.data_model = lora->minfo.mdm_model;
	lora->context.data_revision = lora->minfo.mdm_revision;
	lora->context.data_imei = lora->minfo.mdm_imei;

	/* modem interface */
	const struct modem_iface_uart_config uart_config = {
		.rx_rb_buf = &lora->lora_at_rx_rb_buf[0],
		.rx_rb_buf_len = sizeof(lora->lora_at_rx_rb_buf),
		.dev = DEVICE_DT_GET(LORA_AT_UART_NODE),
		.hw_flow_control = DT_PROP(LORA_AT_UART_NODE, hw_flow_control),
	};


	r = modem_iface_uart_init(&lora->context.iface, &lora->lora_at_data, &uart_config);
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

	lora->conn_status = false;
	lora->state =LORA_STOP;
	lora->lora_con_status =0;
	return 0;
}

int lora_at_config(const struct device *dev,char *data)
{
	int ret;

	char cdeveui[sizeof("AT+CDEVEUI=\"#################\"")] = {0};
	char appeuid[sizeof("AT+CAPPEUI=\"#################\"")] = {0};

	snprintk(cdeveui, sizeof(cdeveui), "AT+CDEVEUI=0000000%s",data);
	snprintk(appeuid, sizeof(appeuid), "AT+CAPPEUI=0000000%s",data);

	ret = modem_cmd_send(&lora.context.iface,
				&lora.context.cmd_handler,
				NULL, 0U,
				"AT+CAPPKEY="APPKEY,
				&lora.sem_response,
				LORA_AT_CMD_SETUP_TIMEOUT);
	ret = modem_cmd_send(&lora.context.iface,
				&lora.context.cmd_handler,
				NULL, 0U,
				appeuid,
				&lora.sem_response,
				LORA_AT_CMD_SETUP_TIMEOUT);
	ret = modem_cmd_send(&lora.context.iface,
			&lora.context.cmd_handler,
			NULL, 0U,
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
	LOG_INF("cdeveui: %s\r\n appeuid:%s",cdeveui,appeuid);
	return ret;
}

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
	lora_at_lock(d);
	if(lora.conn_status == true)
	{


		char buf[data_len +19];
		snprintk(buf, sizeof(buf), "AT+DTRX=%d,%d,%d,%s", CONFIRM,NBTRIALS,data_len,data);

		ret = modem_cmd_send(&lora.context.iface,
					&lora.context.cmd_handler,
					&response_cmds[0],0U,
					buf,
					&lora.sem_response,
					LORA_AT_CMD_SETUP_TIMEOUT);
			lora_at_unlock(d);
		return ret;


	}else{
		LOG_INF("No connection");
			lora_at_unlock(d);
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


	int ret = modem_iface_uart_init_dev(&lora->context.iface,
				DEVICE_DT_GET(LORA_AT_UART_NODE));
	if(ret<0)
	{
		LOG_DBG("ERROR %s Lora","init");
		return ret;
	}
	if(lora->state !=LORA_STOP)
	{
		LOG_ERR("lora_at is already %s", "started");
		goto unlock;
	}

	lora->state = LORA_START;
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lora_pwr_en), okay)
		//TURN_DO_(lora_pwr_en, _ON);
		LOG_WRN("WAITING");
		k_msleep(2000);
 #endif

	if(lora_info == false)
	{
	lora_query_modem_info(lora);
	lora_rejoin();
	lora_info= true;
	// k_work_reschedule(&lora->status_dwork,K_SECONDS(8));
	}
	// int d =lora_flag_set(dev,LORA_CONNECTING);
	// lora_rejoin();
	// lora->lora_status(LORA_EVT_STATED);

unlock:
lora_at_unlock(lora);
}

void lora_stop(const struct device *dev)
{
	struct lora_modem *lora = dev->data;
	lora_at_lock(lora);
	if(lora->state ==LORA_STOP)
	{
		LOG_ERR("lora_at is already %s", "stop");
		lora_at_unlock(lora);
		return;
	}
	if (lora->conn_status!=true)
	{
		lora_at_unlock(lora);
		LOG_ERR("lora_at is still  %s", "not connected");
		return;
	}
// lora->lora_status(LORA_EVT_STOP);
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lora_pwr_en), okay)
	//TURN_DO_(lora_pwr_en, _OFF);
#endif
	lora->state = LORA_STOP;

	lora_at_unlock(lora);

}


DEVICE_DT_DEFINE(DT_INST(0, ebyte_e78), lora_at_init, NULL, &lora, NULL,
		 POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,&lora_at_api);


