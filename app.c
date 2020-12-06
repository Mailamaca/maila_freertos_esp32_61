#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <maila_msgs/msg/esp32_data.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_types.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "driver/i2c.h"
#endif

#include "MadgwickAHRS.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"

#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
maila_msgs__msg__Esp32Data mailamsg;

#define ENCODERS 5
int16_t prev_ticks[ENCODERS] = {};
int16_t ticks;

#define IMU_N_DATA 9
vector_t va, vg, vm;
uint64_t imu_readings = 0;

calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
    .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
    .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},

    .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}};


/**
 * Transformation:
 *  - Rotate around Z axis 180 degrees
 *  - Rotate around X axis -90 degrees
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

/**
 * Transformation: to get magnetometer aligned
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -y;
  v->y = z;
  v->z = -x;
}

void setPCNTParams(int pinPulse,
                 int pinCtrl,
                 pcnt_channel_t channel,
                 pcnt_unit_t unit,
                 uint16_t filter) {

	pcnt_config_t pcnt_config;
      	pcnt_config.pulse_gpio_num = pinPulse;
      	pcnt_config.ctrl_gpio_num = pinCtrl;
      	pcnt_config.channel = channel;
      	pcnt_config.unit = unit;
	pcnt_config.pos_mode = PCNT_COUNT_INC;
	pcnt_config.neg_mode = PCNT_COUNT_DIS;
	pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
	pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
	pcnt_config.counter_h_lim = INT16_MAX;
	pcnt_config.counter_l_lim = 0;
	pcnt_unit_config(&pcnt_config);
	
	pcnt_set_filter_value(unit, filter);
	pcnt_filter_enable(unit);

	gpio_pulldown_en(pinPulse);
	gpio_pulldown_en(pinCtrl);
	
	pcnt_counter_pause(unit);
	pcnt_counter_clear(unit);
	pcnt_counter_resume(unit);

}

int16_t getPCNTDelta(int16_t prev_value, int16_t new_value) {
	
	if (new_value >= prev_value) {
		return (new_value - prev_value);
	} else {
		return (INT16_MAX - prev_value + new_value);
	}
}

void publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

	//RCLC_UNUSED(last_call_time);
	if (timer == NULL) {
		return;
	}
	
	// fill mailamsg.stamp
	mailamsg.stamp.sec = last_call_time / RCL_MS_TO_NS(1000);
	mailamsg.stamp.nanosec = last_call_time - (mailamsg.stamp.sec*RCL_MS_TO_NS(1000));

	

	// encoders
	for (int i=0; i < ENCODERS; i++) {
		pcnt_get_counter_value(i, &ticks);
		mailamsg.int_data.data[i] = getPCNTDelta(prev_ticks[i], ticks);	
		prev_ticks[i] = ticks;
		if (i==0) break; // TODO: remove this for updating all the encoders
	}

	mailamsg.int_data.data[1] = 0;
	mailamsg.int_data.data[2] = 0;
	mailamsg.int_data.data[3] = 0;
	mailamsg.int_data.data[4] = imu_readings;

	
	
	// imu data
	if (imu_readings > 0) {
		mailamsg.float_data.data[0] = va.x / imu_readings;
		mailamsg.float_data.data[1] = va.y / imu_readings;
		mailamsg.float_data.data[2] = va.z / imu_readings;
		mailamsg.float_data.data[3] = vg.x / imu_readings;
		mailamsg.float_data.data[4] = vg.y / imu_readings;
		mailamsg.float_data.data[5] = vg.z / imu_readings;
		mailamsg.float_data.data[6] = vm.x / imu_readings;
		mailamsg.float_data.data[7] = vm.y / imu_readings;
		mailamsg.float_data.data[8] = vm.z / imu_readings;

		imu_readings = 0;
		va.x = va.y = va.z = 0;
		vg.x = vg.y = vg.z = 0;
		vm.x = vm.y = vm.z = 0;
	}	

	// send msg
	RCSOFTCHECK(rcl_publish(&publisher, &mailamsg, NULL));
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

	//RCLC_UNUSED(last_call_time);
	if (timer == NULL) {
		return;
	}

	vector_t _va, _vg, _vm;

	// Get the Accelerometer, Gyroscope and Magnetometer values.
    	ESP_ERROR_CHECK(get_accel_gyro_mag(&_va, &_vg, &_vm));
	//ESP_ERROR_CHECK(get_accel_gyro(&_va, &_vg));
		
	// Transform these values to the orientation of our device.
	transform_accel_gyro(&_va);
	transform_accel_gyro(&_vg);
	transform_mag(&_vm);

	va.x += _va.x;
	va.y += _va.y;
	va.z += _va.z;
	vg.x += _vg.x;
	vg.y += _vg.y;
	vg.z += _vg.z;
	vm.x += _vm.x;
	vm.y += _vm.y;
	vm.z += _vm.z;
	imu_readings++;

}



void appMain(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "maila_freertos_esp32_61", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(maila_msgs, msg, Esp32Data),
		"maila_freertos_esp32_61"));

	// prepare mailamsg.int_data
	mailamsg.int_data.data = (int16_t *)calloc(ENCODERS, sizeof(int16_t));
	mailamsg.int_data.capacity = ENCODERS;
	mailamsg.int_data.size = ENCODERS;
	// prepare mailamsg.float_data
	mailamsg.float_data.data = (float *)calloc(IMU_N_DATA, sizeof(float));
	mailamsg.float_data.capacity = IMU_N_DATA;
	mailamsg.float_data.size = IMU_N_DATA;

	// create publisher_timer
	rcl_timer_t publisher_timer = rcl_get_zero_initialized_timer();
	const unsigned int publisher_timer_timeout = 10000;
	RCCHECK(rclc_timer_init_default(
		&publisher_timer,
		&support,
		RCL_MS_TO_NS(publisher_timer_timeout),
		publisher_timer_callback));

	
	// create imu_timer
	rcl_timer_t imu_timer = rcl_get_zero_initialized_timer();
	const unsigned int imu_timer_timeout = 1;
	RCCHECK(rclc_timer_init_default(
		&imu_timer,
		&support,
		RCL_MS_TO_NS(imu_timer_timeout),
		imu_timer_callback));
	

	// config pcnt
	setPCNTParams(GPIO_NUM_32,GPIO_NUM_33, PCNT_CHANNEL_0, PCNT_UNIT_0, 1); // encoder 0

	// config imu mpu9250
	i2c_mpu9250_init(&cal);
	//MadgwickAHRSinit(500, 0.8); // calc orientation on the raspberry
	
	// create executor
	rclc_executor_t executor;
	executor = rclc_executor_get_zero_initialized_executor();
	unsigned int num_handles = 2 + 0; //n_timers + n_subscriptions;
	RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &publisher_timer));

	/*while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		//usleep(100000);
	}*/

	// spin forever
	rclc_executor_spin(&executor);

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}
