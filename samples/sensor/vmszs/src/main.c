/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <device.h>
#include <sys/printk.h>
#include <drivers/sensor.h>
#include <drivers/sensor/vmszs.h>
#include <settings/settings.h>
#include <shell/shell.h>
#include <sys/reboot.h>
#include <sys/ring_buffer.h>
#include <drivers/gpio.h>
#include <init.h>
#include <math.h>
#include <sys/util.h>
#include <inttypes.h>
#include <drivers/sensor/kx022.h>
#include <sys/crc.h>

#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

#define PI		3.14159265358979323846
#define GRAVITY  9.80665
#define SAMPLE_ANGLE_BUFF 60
#define CALIBRATING_BUFF  60
#define RB_BUF_SIZE	0x00000200
#define RAD_TO_DEG 57.295779513082320876798154814105
#define GPIOS_PORT(do) DT_GPIO_LABEL(DT_NODELABEL(do), gpios)
#define GPIOS_PIN(do) DT_GPIO_PIN(DT_NODELABEL(do), gpios)
#define GPIOS_FLAGS(do) DT_GPIO_FLAGS(DT_NODELABEL(do), gpios)
#define ANGLE_OFFSET 0.05
#define _ON true
#define _OFF false
#define TURN_DO_(do, on)                                                       			   \
	gpio_pin_configure(device_get_binding(GPIOS_PORT(do)),                     		   \
					   GPIOS_PIN(do),                                          \
					   (on ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE) |      \
						   GPIOS_FLAGS(do))

struct k_timer trigger_timer;
struct k_sem trigger_sem;
struct sensor_value val[1];
struct sensor_value temp, hum;
struct sensor_value accel[3];

 int toff = 0;
 int hoff = 0;
 int soff = 0;

static uint64_t timer;
static double_t mbuff[3];
static float_t v[3];
#define DATA_READY_BUFF_SIZE 50

typedef struct Data_Ready{
	int16_t x[DATA_READY_BUFF_SIZE];
	int16_t y[DATA_READY_BUFF_SIZE];
	int16_t z[DATA_READY_BUFF_SIZE];
	int64_t timestamp;

}Data_Ready_t;

typedef struct steady_state{
	int16_t x;
	int16_t y;
	int16_t z;
}steady_state_t;

typedef struct accel_peak{
	int16_t x;
	int16_t y;
	int16_t z;
}accel_peak_t;

typedef struct sample_angle
{
	double_t ax[SAMPLE_ANGLE_BUFF];
	double_t ay[SAMPLE_ANGLE_BUFF];
	double_t az[SAMPLE_ANGLE_BUFF];
}sample_angle_t;

static struct {
	double_t mx,my,mz;
	double_t sx,sy,sz;

}acc_mean;

static sample_angle_t sample_ang;
static Data_Ready_t drdy;
static Data_Ready_t xfer_data;
static steady_state_t steady;
static accel_peak_t peak;
static bool STEADY_STATE_CALIBRATE = false;
static bool xfer_drdy = false;
struct k_sem drdy_sem;


static struct test_context
{
	struct k_work_delayable angle_sample_work;
	double_t angle;
	double_t angleX;
	double_t angleY;
	double_t angleZ;
	double_t originX;
	double_t originY;
	double_t originZ;
	double_t medianX;
	double_t medianY;
	double_t medianZ;
	double_t omedianX;
	double_t omedianY;
	double_t omedianZ;
	double_t velocity;
	double_t o_velocity;
	bool v_1st;
	double_t v_peak[3];

}ctx;

#define ACCEL_DEF_TIMER 60

struct last_axis_t{
	double_t x,y,z;
	double_t cx,cy,cz;
};

// struct accel_rb{
// 	struct ring_buf rb;
// 	uint32_t buffer[RB_BUF_SIZE];
// };



static struct last_axis_t last_axis;

// struct accel_rb a_rb;
uint16_t data_store[50];
static bool first_data = false;

struct settings_read_callback_params
{
	bool value_found;
	void *value;
};

RING_BUF_ITEM_DECLARE_POW2(a_rb,8);

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	memset(&last_axis,0,sizeof(last_axis));
	memset(&mbuff,0,sizeof(mbuff));
	first_data = false;
	// printk("Button pressed at %" PRIu3222 first data: %d\n", k_cycle_get_32(),first_data);

	printk("Button pressed at %" PRIu32 " \tfirst data: %d\n", k_cycle_get_32(),first_data);
}

void button_init(void)
{
	int ret;

	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);

		if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

}
static int settings_read_callback(const char *key,
								  size_t len,
								  settings_read_cb read_cb,
								  void *cb_arg,
								  void *param)
{
	ssize_t num_read_bytes = MIN(len, 32);
	struct settings_read_callback_params *params = param;

	/* Process only the exact match and ignore descendants of the searched name
	 */
	if (settings_name_next(key, NULL) != 0)
	{
		return 0;
	}

	params->value_found = true;
	num_read_bytes = read_cb(cb_arg, params->value, num_read_bytes);

	return 0;
}

int config_save(char *settings_name, void *value, size_t len)
{
	return settings_save_one(settings_name, value, len);
}

int config_apply(char *settings_name, void *value)
{
	int err;
	struct settings_read_callback_params params = { .value_found = false,
													.value = value };

	err = settings_load_subtree_direct(settings_name,
									   settings_read_callback,
									   &params);

	return 0;
}

int config_init(void)
{
	int rc;

	rc = settings_subsys_init();
	if (rc)
	{
		return rc;
	}
		config_apply("sensor/temp", &toff);
		config_apply("sensor/humid", &hoff);
		config_apply("sensor/sound", &soff);

	config_save("sensor/temp", &toff, sizeof(toff));
	config_save("sensor/humid", &hoff, sizeof(hoff));
	config_save("sensor/sound", &soff, sizeof(soff));


	return 0;
}

void accel_rdry_handler(void)
{
	int ret;
	static int i = 0;
	struct sensor_value axis[3];
	const struct device *sensor = device_get_binding("KX022");
	static int k;
	static float tv =0,time,dt =0,test;
	ret =  sensor_sample_fetch_chan(sensor, SENSOR_CHAN_ACCEL_XYZ);

	if (ret == 0) {
		ret = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, axis);
	}

	time = (k_uptime_get_32()-dt);

	v[0] = (time*(sensor_value_to_double(&axis[0])));
	// v[1] = (time*(sensor_value_to_double(&axis[1])));
	// v[2] = (time*(sensor_value_to_double(&axis[2])));
	v[1] = 	pow(sensor_value_to_double(&axis[0]),2)
		+
		pow(sensor_value_to_double(&axis[1]),2)
		+
		pow(sensor_value_to_double(&axis[2]) - 9.8 ,2);

	v[2] = sqrt(v[1])/(time*MSEC_PER_SEC);

	// v[1] =(sensor_value_to_double(&axis[1])*9806.65)/314.16;
	// v[0] =(sensor_value_to_double(&axis[0])*980665)/314.16;
	// test =(sensor_value_to_double(&axis[2])*9806.65)/314.16;
	// v[2] = (v[2]  *9806.65)/314.16;
	// if(fabsf(v[1])>fabsf(test))
	// {
	// 	test = v[1];
	// }
	// printf("/*VX:%f\tVY:%f\tVZ:%f\tV2:%fmm/s\tdiff:%f*/\r\n",
	// sqrt(pow(sensor_value_to_double(&axis[0]),2) /(time*MSEC_PER_SEC)),
	// sqrt(pow(sensor_value_to_double(&axis[1]),2) /(time*MSEC_PER_SEC)),
	// sqrt(pow(sensor_value_to_double(&axis[2]) - 9.8,2) /(time*MSEC_PER_SEC)),
	// v[2]*1000,v[2]*1000 -test);


	// printf("vx :%f\tvy:%f\tvz:%f\r\n",ctx.v_peak[0],ctx.v_peak[1],ctx.v_peak[2]);
	test = v[2] *1000;
	dt = k_uptime_get_32();
	drdy.x[i] = (int)(sensor_value_to_double(&axis[0]) *10000);
	drdy.y[i] = (int)(sensor_value_to_double(&axis[1]) *10000);
	drdy.z[i] = (int)(sensor_value_to_double(&axis[2]) *10000);
	drdy.timestamp = k_uptime_get_32();
	i++;
	if( i == DATA_READY_BUFF_SIZE )
	{
		memcpy(&xfer_data,&drdy,sizeof(drdy));
		xfer_drdy = true;
		i = 0;
	}
}
void accel_motion_handler(void)
{
	printf("Motion detected\r\n");
}
double angle_cal(struct sensor_value *axis)
{
	double_t dot,pro,sum;
	double_t powA,powB,magA,magB,angle;
	double_t cr;
	dot = 	(last_axis.x * last_axis.cx) +
	      	(last_axis.y * last_axis.cy) +
		(last_axis.z * last_axis.cz);

	powA =	pow(last_axis.x,2)+pow(last_axis.y,2)+pow(last_axis.z,2);

	powB =	pow(last_axis.cx,2) +
		pow(last_axis.cy,2) +
		pow(last_axis.cz,2);


	magA =	sqrt(powA);

	magB = 	sqrt(powB);

	// printf("magA %f \magB %f\r\n",(-1*pow(last_axis.cz,2)),pow(last_axis.cz,2));
	 return (acos(dot/(magA*magB)))*57.2958 ;

}
void accel_timer_handler(void)
{
	int count = 0;
	uint8_t size = 49;
	float_t vibration;
	int dt;
	static double_t tmp_v;
	static double_t origin;
	static uint8_t Origin_tmp = 0;

	if(xfer_drdy == true)
	{
		if(STEADY_STATE_CALIBRATE == false)
		{
			for (count =0;count<50;count++)
			{
				if(xfer_data.x[size-count] >steady.x)
				{
					steady.x = xfer_data.x[size-count];
					// printf("\rsteady state x: %d\r\n",steady.x);
				}
				if(xfer_data.y[size-count] >steady.y)
				{
					steady.y = xfer_data.y[size-count];
					// printf("\rsteady state y: %d\r\n",steady.y);
				}
				if((fabs)(xfer_data.z[size-count])>steady.z)
				{
					steady.z = (fabs)(xfer_data.z[size-count]);
					// printf("\rsteady state z: %d\r\n",steady.z);
				}
			}
			STEADY_STATE_CALIBRATE = true;
		}

	// }
		else
		{
			for (count =0;count<50;count++)
			{
				if(xfer_data.x[size-count] >steady.x)
				{
					peak.x = xfer_data.x[size-count];

				}
				if(xfer_data.y[size-count] >steady.y)
				{
					peak.y = xfer_data.y[size-count];
					// printf("\rpeak y: %d\r\n",peak.y);
				}
				if((fabs)(xfer_data.z[size-count])>steady.z)
				{
					peak.z = (fabs)(xfer_data.z[size-count]);
					// printf("\rpeak z: %d\r\n",peak.z);
				}
				vibration =(float)sqrt((pow(xfer_data.x[size-count],2) + pow(xfer_data.y[size-count],2) + pow(xfer_data.z[size-count],2)));
				// dt = xfer_data.timestamp -ddt;
				vibration = (vibration/(2*PI*0.02))/10000;//(vibration *0.02);///10000;
				if(ctx.v_1st == false)
				{

					origin +=vibration;
					Origin_tmp++;

					if(Origin_tmp == 50)
					{
						ctx.o_velocity = origin/50;
						ctx.v_1st = true;
						Origin_tmp = 0;
					}
				}
				else{
				// printf("vibrationx %f\t DV %f\r\n",vibration,tmp_v-vibration);
				ctx.velocity = vibration;
				tmp_v = vibration;
				}
			}
			xfer_drdy = false;
		// vibration = ((double)peak.x*(0.02));
		// printf("peak x: %d\tpeak y: %d\tpeak z: %d\r\n",peak.x,peak.y,peak.z);

		}
	}

}




static double qsort_cmp(const void *a,const void *b)
{
	double aa = *(const double *)a;
	double bb = *(const double *)b;

	return (aa > bb) - (aa < bb);
}

void findMedian(void)
{
	qsort(sample_ang.ax,ARRAY_SIZE(sample_ang.ax),sizeof(double),qsort_cmp);
	qsort(sample_ang.ay,ARRAY_SIZE(sample_ang.ay),sizeof(double),qsort_cmp);
	qsort(sample_ang.az,ARRAY_SIZE(sample_ang.az),sizeof(double),qsort_cmp);

	if(SAMPLE_ANGLE_BUFF % 2 ==0)
	{
		ctx.medianX = (sample_ang.ax[SAMPLE_ANGLE_BUFF/2]) - ctx.originX;
		ctx.medianY = (sample_ang.ay[SAMPLE_ANGLE_BUFF/2]) - ctx.originY;
		ctx.medianZ = (sample_ang.az[SAMPLE_ANGLE_BUFF/2]) - ctx.originZ;
	}
	else{
		ctx.medianX = ((sample_ang.ax[SAMPLE_ANGLE_BUFF/2] - sample_ang.ax[SAMPLE_ANGLE_BUFF/2-1])/2.0) - ctx.originX;
		ctx.medianY = ((sample_ang.ay[SAMPLE_ANGLE_BUFF/2] - sample_ang.ay[SAMPLE_ANGLE_BUFF/2-1])/2.0) - ctx.originY;
		ctx.medianZ = ((sample_ang.az[SAMPLE_ANGLE_BUFF/2] - sample_ang.az[SAMPLE_ANGLE_BUFF/2-1])/2.0) - ctx.originZ;
	}

}
void findMean(void)
{
	uint8_t j =0;

	for(j=0;j<SAMPLE_ANGLE_BUFF;j++)
			{
				acc_mean.sx += sample_ang.ax[j] - ctx.originX;
				acc_mean.sy += sample_ang.ay[j] - ctx.originY;
				acc_mean.sz += sample_ang.az[j] - ctx.originZ;

			}

			ctx.angleX = acc_mean.sx/SAMPLE_ANGLE_BUFF;
			ctx.angleY = acc_mean.sy/SAMPLE_ANGLE_BUFF;
			ctx.angleZ = acc_mean.sz/SAMPLE_ANGLE_BUFF;

}
double round_up(double val)
{
	double_t ret;

	ret = (int)(val *10000 +0.5);

	return ret/10000;
}

double round_2up(double val)
{
	double_t ret;

	ret = (int)(val *10+0.05);

	return ret/10;
}
void accel_work(struct k_work *work)
{
	int ret;
	uint8_t gettype;
	uint32_t getdata[3];
	uint8_t getsize, getval;
	static struct sensor_value ag[3];
	double_t tilt_x,tilt_y,tilt_z;
	double_t pitch,roll,yaw;
	static double_t angle,tilt_x0,tilt_y0,tilt_z0;
	static uint8_t i =0;
	static uint8_t cal =0;
	float rx;
	int rxi;

	const struct device *sensor = device_get_binding("KX022");

	if (!device_is_ready(sensor)) {
		printf("Device kx022 device 1 is not ready\n");
		// sys_reboot(1);
		return 0;
	}
	ret =  sensor_sample_fetch_chan(sensor, SENSOR_CHAN_ACCEL_XYZ);

	if (ret == 0) {
		ret = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, ag);
	}


	// rx =round_up(sensor_value_to_double(&ag[0]));
	// printf("sensor value %f, round up %f\r\n",sensor_value_to_double(&ag[0]),rx);
	if(first_data == false )
	{
		// printf("Calibrating %d.....\r\n",cal);

		last_axis.x += sensor_value_to_double(&ag[0]);
		last_axis.y += sensor_value_to_double(&ag[1]);
		last_axis.z += sensor_value_to_double(&ag[2]);
		cal++;
		if(cal == CALIBRATING_BUFF)
		{
			memset(&sample_ang,0,sizeof(sample_ang));

			last_axis.x/CALIBRATING_BUFF;
			last_axis.y/CALIBRATING_BUFF;
			last_axis.z/CALIBRATING_BUFF;

			ctx.originX = atan(last_axis.x/sqrt(pow(round_up(last_axis.y),2)+(pow(round_up(last_axis.z),2)))) *RAD_TO_DEG;
			ctx.originY = atan(last_axis.y/sqrt(pow(round_up(last_axis.x),2)+(pow(round_up(last_axis.z),2)))) *RAD_TO_DEG;
			ctx.originZ = atan(last_axis.z/sqrt(pow(round_up(last_axis.x),2)+(pow(round_up(last_axis.y),2)))) *RAD_TO_DEG;
			first_data = true;
			cal =0;

		}

	}
	else{
		last_axis.cx = sensor_value_to_double(&ag[0]);
		last_axis.cy = sensor_value_to_double(&ag[1]);
		last_axis.cz = sensor_value_to_double(&ag[2]);
		angle +=angle_cal(&ag);
		// plane_angle();

		sample_ang.ax[i] = atan(last_axis.cx/sqrt(pow(round_up(last_axis.cy),2)+(pow(round_up(last_axis.cz),2)))) *RAD_TO_DEG; //pitch
		sample_ang.ay[i] = atan(last_axis.cy/sqrt(pow(round_up(last_axis.cx),2)+(pow(round_up(last_axis.cz),2)))) *RAD_TO_DEG; //roll
		sample_ang.az[i] = atan(last_axis.cz/sqrt(pow(round_up(last_axis.cx),2)+(pow(round_up(last_axis.cy),2)))) *RAD_TO_DEG; //yaw


		// printf("Angle from Origin :%.4f \tRaw angle[ x %.4f\ty%.2f\tz%.4f] \tRaw angle Origin[ x %.4f\ty%.2f\tz%.4f] \r\n",
		// 	ctx.angle,sample_ang.ax[i],sample_ang.ay[i],sample_ang.az[i],
		// 	sample_ang.ax[i] - tilt_x0,
		// 	sample_ang.ay[i] -tilt_y0,
		// 	sample_ang.az[i] -tilt_z0);


		i++;

		if(i == SAMPLE_ANGLE_BUFF)
		{

			findMedian();
			findMean();
			ctx.angle = angle/SAMPLE_ANGLE_BUFF;
			// printf("\r angle origin: %f angle each axis[ x %.4f\ty%.2f\tz%.4f]\r\n",
			// ctx.angle,ctx.angleX,ctx.angleY,ctx.angleZ);
			memset(&angle,0,sizeof(angle));
			memset(&acc_mean,0,sizeof(acc_mean));
			i = 0;

		}
	}

	k_work_reschedule(&ctx.angle_sample_work,K_MSEC(ACCEL_DEF_TIMER));
	// i++;
	// ret = ring_buf_item_get(&a_rb,&gettype,&getval,getdata,getsize);

	// printk("accel timer handler %f\r\n",getdata[0]);

}
void accel_sample(void)
{
	const struct device *sensor = device_get_binding("KX022");
	int rc;

	if (!device_is_ready(sensor)) {
		printf("Device kx022 device 1 is not ready\n");
		return 0;
	}

	rc = sensor_sample_fetch_chan(sensor, SENSOR_CHAN_ACCEL_XYZ);

	if (rc == 0) {
		rc = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, accel);
	}
}
void sound(void)
{
	const struct device *dev =device_get_binding(DT_LABEL(DT_INST(0, vemsee_vmszs)));

	// int ret;


	if (!device_is_ready(dev)) {
		printk("sensor: device not found.\n");
		// sys_reboot(1);
		return 0;
	}
	// while (1) {
	sensor_sample_fetch(dev);
	if (sensor_channel_get(dev, SENSOR_CHAN_NOISE, val) != 0) {
			printk("sensor: channel get fail.\n");
			// return;
		}
	// printk("Sound %.2f dbA\r\n",sensor_value_to_double(&val[0]));
	// }

}

void sht3x(void)
{
	const struct device *dev = device_get_binding("SHT3XD");
	struct sensor_value last_temp, last_hum;

	int rc;


	if (!device_is_ready(dev)) {
		printk("\nCould not get SHT3XD device\n");
		// sys_reboot(1);
		return 0;
	}

	// printk(" get SHT3XD device\n");

	rc = sensor_sample_fetch(dev);
		if (rc == 0) {
			rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
						&temp);
			last_temp = temp;
		}
		if (rc == 0) {
			rc = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY,
						&hum);
			last_hum = hum;
		}
		if (rc != 0) {
			// printf("SHT3XD: failed: %d\n", rc);
			// hum = last_hum;
			// temp =last_temp;

		}
	// printk("/SHT3XD,%.2f,%0.2f/",
	// 	       sensor_value_to_double(&temp),
	// 	       sensor_value_to_double(&hum));


}

static uint16_t checksum(const uint8_t *data, uint8_t rd_size)
{
	static uint16_t table[256] = {
		0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0,
		0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41,
		0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0,
		0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
		0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1,
		0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341,
		0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1,
		0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
		0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0,
		0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940,
		0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1,
		0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
		0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0,
		0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740,
		0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0,
		0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
		0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0,
		0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541,
		0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0,
		0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
		0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0,
		0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40,
		0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1,
		0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
		0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0,
		0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
	};

	uint8_t xor = 0;
	uint16_t crc = 0xFFFF;

	while (rd_size--) {
		xor = (*data++) ^ crc;
		crc >>= 8;
		crc ^= table[xor];
	}

	return crc;
}

static uint16_t MODBUS_CRC16_v2( const unsigned char *buf, unsigned int len )
{
	static const uint16_t table[2] = { 0x0000, 0xA001 };
	uint16_t crc = 0xFFFF;
	unsigned int i = 0;
	char bit = 0;
	unsigned int xor = 0;

	for( i = 0; i < len; i++ )
	{
		crc ^= buf[i];

		for( bit = 0; bit < 8; bit++ )
		{
			xor = crc & 0x01;
			crc >>= 1;
			crc ^= table[xor];
		}
	}

	return crc;
}


int modbus_test(uint16_t dev_id,uint16_t addr ,uint16_t size)
{
	uint8_t convert[4];
	uint8_t test[6]={dev_id,0x03,(addr>>8 &0xff),(addr>>0 &0xff),(size>>8 &0xff),(size>>0 &0xff)};
	// uint8_t test[6] = { 0x04, 0x03, 0x00, 0x00, 0x00, 0x01};
	return MODBUS_CRC16_v2(&test,sizeof(test));
}


int main(void)
{
	struct sensor_trigger trig;
	struct sensor_trigger trig_motion;
	uint8_t rc;
	int16_t crc;
	const struct device *sensor = device_get_binding("KX022");
	const struct device *sensor1 = device_get_binding("KX022_2");
	double_t lastmedX,lastmedY,lastmedZ;
	
	// int j =0;
	// uint8_t gettype;
	// uint32_t getdata[3];
	// uint8_t getsize, getval;

	trig.type = SENSOR_TRIG_DATA_READY;
	ctx.v_1st = false;

	trig_motion.type = SENSOR_TRIG_KX022_MOTION;

	button_init();
	memset(&ctx.v_peak,0,sizeof(ctx.v_peak));

	rc = sensor_trigger_set(sensor, &trig, accel_rdry_handler);

	rc = sensor_trigger_set(sensor1, &trig_motion, accel_motion_handler);

	config_init();

	k_timer_init(&trigger_timer,accel_timer_handler,NULL);
	k_timer_start(&trigger_timer,K_MSEC(ACCEL_DEF_TIMER),K_MSEC(ACCEL_DEF_TIMER));

	// ring_buf_init(&a_rb,sizeof(a_rb.buffer),a_rb.buffer);

	k_work_init_delayable(&ctx.angle_sample_work,accel_work);
	k_work_reschedule(&ctx.angle_sample_work,K_NO_WAIT);

	// k_sem_init(&drdy_sem,0,1);

	TURN_DO_(fan_switch, _ON);


	while(1)
	{
		crc =modbus_test(4,0x0000,0x0001);

	printf("crc %x\r\c",crc);

			// sound();
			// sht3x();
			accel_sample();

		// if ((fabs(ctx.medianX) <0.05))
		// 	{
		// 		ctx.medianX = 0 ;
		// 		// lastmedX =ctx.medianX;
		// 	}
		// if ((fabs(ctx.medianY) <0.05))
		// 	{
		// 		ctx.medianY = 0 ;
		// 		// lastmedX =ctx.medianX;
		// 	}
		// if ((fabs(ctx.medianZ) <0.05))
		// 	{
		// 		ctx.medianZ = 0 ;
		// 		// lastmedX =ctx.medianX;
		// 	}
		// // // else
		// // // 	{
		// // // 		ctx.medianX = lastmedX;
		// // // 	}
		double_t a,b,c;
		a =fabs(ctx.medianX);
		b = fabs(lastmedX);
		c = a-b;

		if ((fabs(c)>ANGLE_OFFSET))
		{
			mbuff[0] =round_2up(ctx.medianX);
		}

		a =fabs(ctx.medianY);
		b = fabs(lastmedY);
		c = a-b;

		 if((fabs(c)>ANGLE_OFFSET))
		{
			mbuff[1] = round_2up(ctx.medianY);
		}

		a =fabs(ctx.medianZ);
		b = fabs(lastmedZ);
		c = a-b;

		if((fabs(c)>ANGLE_OFFSET))
			{
				mbuff[2] = round_2up(ctx.medianZ);
			}



		// if(first_data == true){
		// 	printk("/*Sensor test,%.2f,%0.2f,%0.2f,%.5f,%0.5f,%.5f,%.5f,%0.3f,%.3f,%.3f,%0.2f,%.2f,%.2f,%f,%f,%f,%f*/\r\n",
		// 		sensor_value_to_double(&temp)+(toff),
		// 		sensor_value_to_double(&hum)+(hoff),
		// 		sensor_value_to_double(&val[0])+(soff),
		// 		sensor_value_to_double(&accel[0]),
		// 		sensor_value_to_double(&accel[1]),
		// 		sensor_value_to_double(&accel[2]),
		// 		ctx.angle,
		// 		ctx.angleX,
		// 		ctx.angleY,
		// 		ctx.angleZ,
		// 		// ctx.medianX,
		// 		// ctx.medianY,
		// 		// ctx.medianZ,
		// 		mbuff[0],
		// 		mbuff[1],
		// 		mbuff[2],//);
		// 		ctx.velocity-ctx.o_velocity,
		// 		ctx.v_peak[0],
		// 		ctx.v_peak[1],
		// 		ctx.v_peak[2]);

		// 	lastmedX =ctx.medianX;
		// 	lastmedY =ctx.medianY;
		// 	lastmedZ = ctx.medianZ;
			// k_sleep(K_MSEC(500));
		// }
	}
}

static int cmd_temp_set(const struct shell *shell, size_t argc, char *argv[])
{

	int temp;
	char *ptr;

	temp = (int)strtol(argv[2], &ptr, 10);

	shell_print(shell, "old temp = %d", toff);
	settings_save_one(argv[1], &temp, sizeof(temp));

	config_apply(argv[1], &toff);
		shell_print(shell, "new temp = %d", toff);

	return 0;
}

static int cmd_hum_set(const struct shell *shell, size_t argc, char *argv[])
{

	int temp;
	char *ptr;

	temp = (int)strtol(argv[2], &ptr, 10);

	shell_print(shell, "old humidity = %d", hoff);
	settings_save_one(argv[1], &temp, sizeof(temp));

	config_apply(argv[1], &hoff);
		shell_print(shell, "new humidity = %d", hoff);

	return 0;
}
static int cmd_sound_set(const struct shell *shell, size_t argc, char *argv[])
{

	int temp;
	char *ptr;

	temp = (int)strtol(argv[2], &ptr, 10);

	shell_print(shell, "old sound = %d", soff);
	settings_save_one(argv[1], &temp, sizeof(temp));

	config_apply(argv[1], &soff);
		shell_print(shell, "new sound = %d", toff);

	return 0;
}


SHELL_STATIC_SUBCMD_SET_CREATE(smp_cmds,
	SHELL_CMD_ARG(tem, NULL, "Set temp offset", cmd_temp_set, 2, 2),
	SHELL_CMD_ARG(hmd, NULL, "Set humidity offset", cmd_hum_set, 2, 2),
	SHELL_CMD_ARG(snd, NULL, "Set sound offset", cmd_sound_set, 2, 2),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(smp, &smp_cmds, "SMP shell commands", NULL);
