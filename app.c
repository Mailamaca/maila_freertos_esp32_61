#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <maila_msgs/msg/tick_delta.h>
#include <maila_msgs/msg/esp32_data.h>

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

rcl_publisher_t data_publisher;
maila_msgs__msg__Esp32Data data_msg;

#define ENCODERS 5
int16_t prev_ticks[ENCODERS] = {};
int16_t delta_ticks[ENCODERS] = {};

vector_t va, vg, vm;
const float G = 9.807f;
const float _d2r = 3.14159265359f/180.0f;
uint64_t imu_readings = 0;

#define FRAME_ID_STRING_LEN 20
const char imu_frame_id[] = "imu";
const char tick_frame_id[] = "encoder";

const unsigned int publisher_timer_timeout = 10; // ms

calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
    .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
    .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},

    .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}
};


void transform_accel(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = x * G;
  v->y = y * G;
  v->z = z * G;
}

void transform_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = x * _d2r;
  v->y = y * _d2r;
  v->z = z * _d2r;
}

void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = y;
  v->y = x;
  v->z = -z;
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

void read_imu()
{

	vector_t _va, _vg, _vm;

	// Get the Accelerometer, Gyroscope and Magnetometer values.
	ESP_ERROR_CHECK(get_accel_gyro_mag(&_va, &_vg, &_vm));
	//ESP_ERROR_CHECK(get_accel_gyro(&_va, &_vg));
			
	// Transform these values to the orientation of our device.
	transform_accel(&_va);
	transform_gyro(&_vg);
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

void read_encoders()
{
	int16_t ticks;
	for (int i=0; i < ENCODERS; i++) {
		pcnt_get_counter_value(i, &ticks);
		delta_ticks[i] = getPCNTDelta(prev_ticks[i], ticks);	
		prev_ticks[i] = ticks;
		if (i==0) break; // TODO: remove this for updating all the encoders ****
	}
	delta_ticks[1] = imu_readings; // debug**********************
	delta_ticks[2] = imu_readings; // debug**********************
	delta_ticks[3] = imu_readings; // debug**********************
	delta_ticks[4] = imu_readings; // debug**********************
}

void prepare_data_msg()
{
	data_msg.int_data.data = (int16_t *) malloc(ENCODERS * sizeof(int16_t));
	data_msg.int_data.capacity = ENCODERS;
	data_msg.int_data.size = ENCODERS;


	data_msg.float_data.data = (float *) malloc(9 * sizeof(float));
	data_msg.float_data.capacity = 9;
	data_msg.float_data.size = 9;
}

void publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	int64_t act_time = esp_timer_get_time(); // us since start
	int64_t act_sec = act_time / 1000000;
	int64_t act_nanosec = (act_time - (act_sec*1000000)) * 1000;

	//RCLC_UNUSED(last_call_time);
	if (timer == NULL) {
		return;
	}
	
	// encoders
	read_encoders();

	// imu
	if (imu_readings <= 0) read_imu();

	// tick_msg
	data_msg.stamp.sec = last_call_time / RCL_MS_TO_NS(1000);
	data_msg.stamp.nanosec = last_call_time - (data_msg.stamp.sec*RCL_MS_TO_NS(1000));
	for (int i=0; i < ENCODERS; i++) {
		data_msg.int_data.data[i] = delta_ticks[i];
	}

	data_msg.float_data.data[0] =  va.x / imu_readings;
	data_msg.float_data.data[1] =  va.y / imu_readings;
	data_msg.float_data.data[2] =  va.z / imu_readings;
	data_msg.float_data.data[3] =  vg.x / imu_readings;
	data_msg.float_data.data[4] =  vg.y / imu_readings;
	data_msg.float_data.data[5] =  vg.z / imu_readings;
	data_msg.float_data.data[6] =  vm.x / imu_readings;
	data_msg.float_data.data[7] =  vm.y / imu_readings;
	data_msg.float_data.data[8] =  vm.z / imu_readings;


	RCSOFTCHECK(rcl_publish(&data_publisher, &data_msg, NULL));
	
	// reset
	imu_readings = 0;
	va.x = va.y = va.z = 0;
	vg.x = vg.y = vg.z = 0;
	vm.x = vm.y = vm.z = 0;
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
		&data_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(maila_msgs, msg, Esp32Data),
		"esp32_61"));
	prepare_data_msg();

	// create publisher_timer
	rcl_timer_t publisher_timer = rcl_get_zero_initialized_timer();
	RCCHECK(rclc_timer_init_default(
		&publisher_timer,
		&support,
		RCL_MS_TO_NS(publisher_timer_timeout),
		publisher_timer_callback));	

	// config pcnt
	setPCNTParams(GPIO_NUM_32,GPIO_NUM_33, PCNT_CHANNEL_0, PCNT_UNIT_0, 1); // encoder 0

	// config imu mpu9250
	i2c_mpu9250_init(&cal);
	//MadgwickAHRSinit(500, 0.8); // calc orientation on the raspberry
	
	// create executor
	rclc_executor_t executor;
	unsigned int num_handles = 1 + 0; //n_timers + n_subscriptions;
	RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &publisher_timer));
	
	while(true) {
		read_imu();
		rclc_executor_spin_some(&executor, 1000); // nanosec
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&data_publisher, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}
