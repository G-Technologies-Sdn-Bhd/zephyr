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
#include <shell/shell.h>
#include <sys/reboot.h>
#include <sys/ring_buffer.h>
#include <math.h>
#include <sys/util.h>
#include <inttypes.h>
#include <drivers/sensor/kx022.h>
#include <sys/crc.h>

K_SEM_DEFINE(accel_data_sem, 0, 1);
K_MUTEX_DEFINE(velocity_mutex);


static struct {
    double velocity[3];
    double last_acceleration[3] ;
    int zero_crossing_count;
    bool is_positive_crossing; 
    bool initial_data_processed; 
    double magnitude;
    double highest_velocity;
    bool veloMon;
    double gravity[3];
    bool evt_start;
}velocity_data;

#define STACK_SIZE 1024
#define PRIORITY_ACCELEROMETER_THREAD 5
#define PRIORITY_FREQUENCY_MONITORING_THREAD 6

static K_THREAD_STACK_DEFINE(accelerometer_thread_stack, STACK_SIZE);
static K_THREAD_STACK_DEFINE(frequency_monitoring_thread_stack, STACK_SIZE);

struct k_thread accelerometer_thread_data;
struct k_thread frequency_monitoring_thread_data;



double mms2_convert2(double val)
{
	// val *= 1000;
	/*convert m/s2 to mm/s*/
	val = (val * 0.01) * 1000;
	return val;//*1000; //(val / FREQUNCY);
}
double mms2_convert(double val)
{
	// val *= 1000;
	/*convert m/s2 to mm/s*/
	// val = (val * 0.01) * 1000;
	return val;//*1000; //(val / FREQUNCY);
}

void read_sensor_data(double accelerometer_data[3]) {
    int ret;
	struct sensor_value axis[3];
    const struct device *sensor = device_get_binding("KX022");
    ret =  sensor_sample_fetch_chan(sensor, SENSOR_CHAN_ACCEL_XYZ);
    if (ret == 0) {
		ret = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, axis);
	}
    // Replace this function with the actual implementation
    accelerometer_data[0] =  mms2_convert(sensor_value_to_double(&axis[0]));  // X-axis acceleration
    accelerometer_data[1] =  mms2_convert(sensor_value_to_double(&axis[1]));  // Y-axis acceleration
    accelerometer_data[2] =  mms2_convert(sensor_value_to_double(&axis[2])); // Z-axis acceleration
}

// Apply a high-pass filter to remove gravity effect
void high_pass_filter(double accelerometer_data[3], double gravity[3], double alpha) {
    for (int i = 0; i < 3; i++) {
        gravity[i] = alpha * gravity[i] + (1 - alpha) * accelerometer_data[i];
        accelerometer_data[i] -= gravity[i];
    }
}

// Process accelerometer data and calculate velocity using kinematic equation
void process_accelerometer_data(double accelerometer_data[3], double velocity[3], double dt) {
    // Integrate acceleration to get velocity
    for (int i = 0; i < 3; i++) {
        velocity_data.velocity[i] = accelerometer_data[i] * dt;  // v = at, assuming constant acceleration
    }
}

// Count zero-crossings to estimate frequency
void count_zero_crossings(double accelerometer_data[3],double threshold) {
    if(threshold >5){
    // velocity_data.evt_start =true;
    for (int i = 0; i < 3; i++) {
        if (accelerometer_data[i] * velocity_data.last_acceleration[i] < 0) {
            // Sign change detected, increment the zero-crossing count
            velocity_data.zero_crossing_count++;

            // Update the sign flag for the next iteration
            velocity_data.is_positive_crossing = accelerometer_data[i] > 0;
        }
    }
    // }
    // Save the current accelerometer data for the next iteration
    memcpy(velocity_data.last_acceleration, accelerometer_data, sizeof(velocity_data.last_acceleration));
    }
}

static void accel_rdry_handler(const struct device *dev,
							   struct sensor_trigger *trig)
{
    int ret;
	struct sensor_value axis[3];


    double accel, x, y, z, data[3], d[3];
	static double data2[3], dv, ddv, v_old, vmax, vrms;
	static bool i;
	static int cnt, normal_cnt;
	int32_t deltaTime;

    	ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);

	if (ret == 0)
	{
		ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, axis);
	}

	data[0] = mms2_convert2(sensor_value_to_double(&axis[0]));
	data[1] = mms2_convert2(sensor_value_to_double(&axis[1]));
	data[2] = mms2_convert2(sensor_value_to_double(&axis[2]));

    d[0] = data2[0] - data[0];
	d[1] = data2[1] - data[1];
	d[2] = data2[2] - data[2];

	data2[0] = data[0];
	data2[1] = data[1];
	data2[2] = data[2];

	x = pow(data[0], 2);
	y = pow(data[1], 2);
	z = pow(data[2], 2);

	accel = sqrt(x + y + z);

    
	dv = accel;

	 velocity_data.magnitude = dv - v_old;

    if(velocity_data.magnitude >velocity_data.highest_velocity &&velocity_data.veloMon ==true)
    {
    velocity_data.highest_velocity  = velocity_data.magnitude;
    }
     
    velocity_data.velocity[0] = data2[0];
    velocity_data.velocity[1] = data2[1];
    velocity_data.velocity[2] = data2[2];
    count_zero_crossings(data2, velocity_data.magnitude);
    // printf("X:%f\tY:%f\tZ:%f\tvel:%f\r\n",
    //   data2[0],data2[1],
    //   data2[2],ddv);

    v_old = dv;
  
}
void accelerometer_thread(void) 
{
  
    double alpha = 0.9;  // Filter coefficient, adjust as needed
    double dt = 0.1;  // Example: Assuming a sampling interval of 0.1 seconds
    // static bool start =false;
    while (1) {
        double accelerometer_data[3];
     double x,y,z;
        // Read accelerometer data
        read_sensor_data(accelerometer_data);
        if (!velocity_data.initial_data_processed) {
                velocity_data.initial_data_processed = true;
                continue;
        }
        // Apply high-pass filter to remove gravity effect
        high_pass_filter(accelerometer_data, velocity_data.gravity, alpha);
       
        // Process accelerometer data using kinematic equation
        // k_mutex_lock(&velocity_mutex, K_FOREVER);
        process_accelerometer_data(accelerometer_data, velocity_data.velocity, dt);
          x= pow(velocity_data.velocity[0],2);
         y=pow(velocity_data.velocity[1],2);
         z=pow(velocity_data.velocity[2],2);
       
         double mag =sqrt(x+y+z)*1000;
         velocity_data.magnitude = mag; 
         
         if(velocity_data.magnitude >velocity_data.highest_velocity &&velocity_data.veloMon ==true)
         {
            velocity_data.highest_velocity  = velocity_data.magnitude;
         }
        count_zero_crossings(accelerometer_data,mag);
        
        // k_mutex_unlock(&velocity_mutex);


        // // Signal completion or perform other tasks
        // k_sem_give(&accel_data_sem);

        // // Sleep or yield as needed
        // k_sleep(K_MSEC(100));
    }
}
static int tHz=0;
static double tppk=0;
// static struct {
//     int tHz;
//     double tppk;
    
// }
void frequency_monitoring_thread(void) {
    static bool start =false;
    while (1) {
        // Wait for accelerometer data processing to complete
        // k_sem_take(&accel_data_sem, K_FOREVER);
        // if(!start){
        //     start = true;
        //      continue;
        // }
    
  
    if(velocity_data.zero_crossing_count!=0 && velocity_data.magnitude >5)
    {
        if(velocity_data.evt_start!= true)
        {
            velocity_data.evt_start= true;
        }

        tHz +=velocity_data.zero_crossing_count;
        if(velocity_data.highest_velocity>tppk)
        {
            tppk= velocity_data.highest_velocity;
        }
    }
    else
    {
       if(velocity_data.evt_start== true)
        {
            // printf("\n\n=================Event Trigger===========\r\n\n");
            velocity_data.evt_start= false;
        }
        tppk=0;
        tHz=0;
    }
    printf("X:%f\tY:%f\tZ:%f\tvel:%f\tpeak:%f\thz:%d\ttotalHz%d\tppk:%f\r\n",
            velocity_data.velocity[0],velocity_data.velocity[1],
            velocity_data.velocity[2],velocity_data.magnitude,
            velocity_data.highest_velocity ,velocity_data.zero_crossing_count,
            tHz,tppk);
    
    

    velocity_data.highest_velocity =0;
    velocity_data.zero_crossing_count =0;
    velocity_data.veloMon =true;
    k_sleep(K_MSEC(1000));  
    }
}

void start_threads(void) {
   k_tid_t tid,ttid;

    static bool start = false;
	struct sensor_trigger trig1 = { .type = SENSOR_TRIG_DATA_READY };
    const struct device *sensor = device_get_binding("KX022");
        
    trig1.type = SENSOR_TRIG_DATA_READY;
    int ret = sensor_trigger_set(sensor, &trig1, accel_rdry_handler);

    	// tid = k_thread_create(&accelerometer_thread_data,
		// 					  accelerometer_thread_stack,
		// 					  K_THREAD_STACK_SIZEOF(accelerometer_thread_stack),
		// 					  (k_thread_entry_t)accelerometer_thread,
		// 					  NULL,
		// 					  NULL,
		// 					  NULL,
		// 					  1,
		// 					  0,
		// 					    K_NO_WAIT);
        ttid = k_thread_create(&frequency_monitoring_thread_data,
                    frequency_monitoring_thread_stack,
                    K_THREAD_STACK_SIZEOF(frequency_monitoring_thread_stack),
                    (k_thread_entry_t)frequency_monitoring_thread,
                    NULL,
                    NULL,
                    NULL,
                    2,
                    0,
                    K_SECONDS(15));
}


void main(void) {
    
    printf("------------------------\r\n");
    // Start threads
    memset(&velocity_data,0,sizeof(velocity_data));
    start_threads();

    //   while (1) {
        // accelerometer_thread();
        // k_sleep(K_SECONDS(5));
    //      k_sleep(K_MSEC(100));
    //   }
}
