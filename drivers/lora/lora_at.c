/*
 * Copyright (c) 2019 Tobias Svehagen
 * Copyright (c) 2020 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "gsm_ppp.h"
#include "modem_context.h"
#include "modem_iface_uart.h"
#include "modem_cmd_handler.h"
#include "../console/gsm_mux.h"
#include <stdio.h>
#include <time.h>

// #include "lora_at.h"

#define LORA_AT_UART_NODE                   DT_INST_BUS(0)
#define LORA_AT_CMD_READ_BUF                128
#define LORA_AT_CMD_AT_TIMEOUT              K_SECONDS(2)
#define LORA_AT_CMD_SETUP_TIMEOUT           K_SECONDS(6)
#define LORA_AT_RX_STACK_SIZE               512//CONFIG_MODEM_LORA_AT_RX_STACK_SIZE
#define LORA_AT_WORKQ_STACK_SIZE           768//CONFIG_MODEM_LORA_AT_WORKQ_STACK_SIZE
#define LORA_AT_RECV_MAX_BUF                30
#define LORA_AT_RECV_BUF_SIZE               128
#define LORA_AT_ATTACH_RETRY_DELAY_MSEC     1000
#define LORA_AT_REGISTER_DELAY_MSEC         1000
#define LORA_AT_REGISTER_TIMEOUT            30//60
#define CONFIRM 1
#define NBTRIALS 2

#define A_PPP_MRU 1500
static struct lora_modem{
	const struct device *dev;
	
	struct modem_context context;

	struct k_mutex lock;
	
	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_match_buf[LORA_AT_CMD_READ_BUF];
	struct k_sem sem_response;

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
	// struct net_if *iface;
}lora;

NET_BUF_POOL_DEFINE(lora_at_recv_pool, LORA_AT_RECV_MAX_BUF, LORA_AT_RECV_BUF_SIZE,
		    0, NULL);
K_KERNEL_STACK_DEFINE(lora_at_rx_stack, LORA_AT_RX_STACK_SIZE);
K_KERNEL_STACK_DEFINE(lora_at_workq_stack, LORA_AT_WORKQ_STACK_SIZE);

// struct lora_at lora_at_data_drv;

// static inline int lora_at_work_reschedule(struct k_work_delayable *dwork, k_timeout_t delay)
// {
// 	if (lora_at.workq_plug) {
// 		return 0;
// 	}

// 	return k_work_reschedule_for_queue(&lora_at.workq, dwork, delay);
// }

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
// static const struct lora_mgmt lora_api = {
	
// };

static void lora_at_rx(struct lora_modem *lora)
{
	// LOG_DBG("starting");
	
	while (true) {
		(void)k_sem_take(&lora->lora_at_data.rx_sem, K_FOREVER);
		/* The handler will listen AT channel */
		lora->context.cmd_handler.process(&lora->context.cmd_handler,
						 &lora->context.iface);
	}
}
MODEM_CMD_DEFINE(lora_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	LOG_DBG("ok");

	k_sem_give(&lora.sem_response);
	return 0;
}
MODEM_CMD_DEFINE(lora_cmd_error)
{
	modem_cmd_handler_set_error(data, -EINVAL);
	LOG_DBG("ok");
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

	return 0;
}

static const struct setup_cmd setup_modem_info_cmds[] ={
		/* query modem info */
	SETUP_CMD("AT+CGMI?", "+CGMI=", on_cmd_atcmdinfo_manufacturer, 0U, ""),
	SETUP_CMD("AT+CGMM?", "+CGMM=", on_cmd_atcmdinfo_model, 0U, ""),
	SETUP_CMD("AT+CGMR?", "+CGMR=", on_cmd_atcmdinfo_revision, 0U, ""),
	SETUP_CMD("AT+CGSN?", "+CGSN=", on_cmd_atcmdinfo_imei, 0U, ""),
	// SETUP_CMD("AT+CCLASS=0","+CCLASS:",on_cmd_atcmdinfo_class, 0U, ""),
};

static int lora_query_modem_info(struct lora_modem *lora)
{
	int ret;

	// if (gsm->modem_info_queried) {
	// 	return 0;
	// }
	ret =  modem_cmd_handler_setup_cmds(&lora->context.iface,
					    &lora->context.cmd_handler,
					    setup_modem_info_cmds,
					    ARRAY_SIZE(setup_modem_info_cmds),
					    &lora->sem_response,
					    K_MSEC(1000));

	if (ret < 0) {
		return ret;
	}

	// gsm->modem_info_queried = true;

	return 0;
}
// static const struct modem_cmd read_joinmod_cmd =
// MODEM_CMD_ARGS_MAX("+COPS:", on_cmd_atcmdjoin, 1U, 4U, ",");

// /*
//  * Handler: +COPS: <mode>[0],<format>[1],<oper>[2]
//  */
// MODEM_CMD_DEFINE(on_cmd_atcmdjoin)
// {
// size_t out_len;

// 	out_len = net_buf_linearize(lora.minfo.mdm_imei, sizeof(lora.minfo.mdm_imei) - 1,
// 				    data->rx_buf, 0, len);
// 	lora.minfo.mdm_imei[out_len] = '\0';
// 	LOG_INF(": %s", log_strdup(lora.minfo.mdm_imei));

// 	return 0;
// }

MODEM_CMD_DEFINE(on_cmd_atcmdjoin)
{
	modem_cmd_handler_set_error(data, 0);
	LOG_INF("Successfully Join");
	lora.conn_status = true;
	k_sem_give(&lora.sem_response);
	return 0;
}
static const struct setup_cmd setup_modem_join[] ={
	SETUP_CMD_NOHANDLE("AT+CCLASS=0"),
	// SETUP_CMD("AT+CCLASS=0","+CCLASS:",on_cmd_atcmdinfo_class, 0U, ""),
	SETUP_CMD_NOHANDLE("AT+CJOINMODE=0"),
	SETUP_CMD_NOHANDLE("AT+CJOIN=1,1,8,8"),
	// SETUP_CMD_NOHANDLE("AT+CJOIN=1,0,8,8","",on_cmd_atcmdjoin,3U,","),
};


static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK", lora_cmd_ok, 0U, ""),
	MODEM_CMD("+CJOIN", on_cmd_atcmdjoin, 0U, ""),
	MODEM_CMD("ERROR", lora_cmd_error, 0U, ""),
	// MODEM_CMD()
};
static int lora_at_init(const struct device *dev)
{
	struct lora_modem *lora = dev->data;
	static const struct setup_cmd setup_cmds[] = {
		SETUP_CMD_NOHANDLE("AT"),
		/* turn off echo */
		// SETUP_CMD_NOHANDLE("ATE0"),
		};
	int r;
	// LOG_DBG("Lora at (%p)", lora);
	
	lora->dev = dev;
	
	(void)k_mutex_init(&lora->lock);

	lora->cmd_handler_data.cmds[CMD_RESP] = response_cmds;
	lora->cmd_handler_data.cmds_len[CMD_RESP] = ARRAY_SIZE(response_cmds);
	// lora_->cmd_handler_data.cmds[CMD_UNSOL] = unsol_cmds;
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
		// LOG_DBG("iface uart error %d", r);
		return r;
	}

	r = modem_context_register(&lora->context);
	if (r < 0) {
		// LOG_DBG("context error %d", r);
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

r =   modem_cmd_handler_setup_cmds(&lora->context.iface,
					&lora->context.cmd_handler,
					setup_modem_join,
					ARRAY_SIZE(setup_modem_join),
					&lora->sem_response,
					LORA_AT_CMD_SETUP_TIMEOUT);

r = lora_query_modem_info(lora);
k_msleep(5000);
	return 0;
	// gsm_ppp_start(dev);
}

int lora_at_config(struct lora_modem *lora)
{
	int ret ;
	ret =  modem_cmd_handler_setup_cmds(&lora->context.iface,
					&lora->context.cmd_handler,
					setup_modem_join,
					ARRAY_SIZE(setup_modem_join),
					&lora->sem_response,
					LORA_AT_CMD_SETUP_TIMEOUT);
	return ret;
}

// static const struct setup_cmd setup_modem_join[] ={
// 	SETUP_CMD_NOHANDLE("AT+DTRX="), 1,2,3,112233
// 	// SETUP_CMD("AT+CJOIN=1,0,8,8","",on_cmd_atcmdjoin,3U,","),
// };
int lora_at_send(const struct device *dev, char *data,
		     uint32_t data_len)
{
	int  ret;
	if(lora.conn_status == true)
	{
		char buf[sizeof("AT+DTRX=\"#\",\"#\",\"##\",\"#################\"")] = {0};
		snprintk(buf, sizeof(buf), "AT+DTRX=%d,%d,%d,%s", CONFIRM,NBTRIALS,data_len,data);
		ret = modem_cmd_send(&lora.context.iface,
					&lora.context.cmd_handler,
					NULL, 0U,
					buf,
					&lora.sem_response,
					LORA_AT_CMD_SETUP_TIMEOUT);
		
	}else{
		LOG_INF("No connection");
		return -1;
	}

	return 0;
}


static const struct lora_driver_api lora_at_api = {
	.config = lora_at_config,
	.send = lora_at_send,
	// .send_async = lora_at_send_async,
	// .recv = lora_at_recv,
	// .test_cw = lora_at_test_cw,
};

DEVICE_DT_DEFINE(DT_INST(0, ebyte_e78), lora_at_init, NULL, &lora, NULL,
		 POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,&lora_at_api);


