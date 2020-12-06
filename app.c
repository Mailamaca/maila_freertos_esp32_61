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

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t mag_publisher;
sensor_msgs__msg__MagneticField mag_msg;

rcl_publisher_t tick_publisher;
maila_msgs__msg__TickDelta tick_msg;

#define ENCODERS 5
int16_t prev_ticks[ENCODERS] = {};
int16_t delta_ticks[ENCODERS] = {};
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

void read_imu()
{

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

void read_encoders()
{
	int16_t ticks;
	for (int i=0; i < ENCODERS; i++) {
		pcnt_get_counter_value(i, &ticks);
		delta_ticks[i] = getPCNTDelta(prev_ticks[i], ticks);	
		prev_ticks[i] = ticks;
		if (i==0) break; // TODO: remove this for updating all the encoders ****
	}
	delta_ticks[4] = imu_readings; // debug**********************
}

void prepare_imu_msg()
{

}

void prepare_mag_msg()
{

}

void prepare_tick_msg()
{

	tick_msg.ticks.data = (int16_t *)calloc(ENCODERS, sizeof(int16_t));
	tick_msg.ticks.capacity = ENCODERS;
	tick_msg.ticks.size = ENCODERS;

}

void publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	//RCLC_UNUSED(last_call_time);
	if (timer == NULL) {
		return;
	}
	
	int64_t act_time = 0; //esp_timer_get_time(); // us since start
	int64_t act_sec = act_time / 1000000;
	int64_t act_nanosec = (act_time - (act_sec*1000000)) * 1000;

	// encoders
	read_encoders();

	// tick_msg
	tick_msg.header.stamp.sec = act_sec;
	tick_msg.header.stamp.nanosec = act_nanosec;
	tick_msg.delta.sec = last_call_time / RCL_MS_TO_NS(1000);
	tick_msg.delta.nanosec = last_call_time - (tick_msg.delta.sec*RCL_MS_TO_NS(1000));
	for (int i=0; i < ENCODERS; i++) {
		tick_msg.ticks.data[i] = delta_ticks[i];
	}
	RCSOFTCHECK(rcl_publish(&tick_publisher, &tick_msg, NULL));

	// imu_msg
	if (imu_readings <= 0) read_imu();
	imu_msg.header.stamp.sec = act_sec;
	imu_msg.header.stamp.nanosec = act_nanosec;
	imu_msg.angular_velocity.x = vg.x / imu_readings;
	imu_msg.angular_velocity.y = vg.y / imu_readings;
	imu_msg.angular_velocity.z = vg.z / imu_readings;
	imu_msg.linear_acceleration.x = va.x / imu_readings;
	imu_msg.linear_acceleration.y = va.y / imu_readings;
	imu_msg.linear_acceleration.z = va.z / imu_readings;
	RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

	// mag_msg
	if (imu_readings <= 0) read_imu();
	mag_msg.header.stamp.sec = act_sec;
	mag_msg.header.stamp.nanosec = act_nanosec;
	mag_msg.magnetic_field.x = vm.x / imu_readings;
	mag_msg.magnetic_field.y = vm.y / imu_readings;
	mag_msg.magnetic_field.z = vm.z / imu_readings;
	RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
	
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

	// create imu publisher
	RCCHECK(rclc_publisher_init_default(
		&imu_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"sensors/imu/data_raw"));
	prepare_imu_msg();

	// create mag publisher
	RCCHECK(rclc_publisher_init_default(
		&mag_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
		"sensors/imu/mag"));
	prepare_mag_msg();

	// create tick publisher
	RCCHECK(rclc_publisher_init_default(
		&tick_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(maila_msgs, msg, TickDelta),
		"sensors/encoders/tick"));
	prepare_tick_msg();	

	// create publisher_timer
	rcl_timer_t publisher_timer = rcl_get_zero_initialized_timer();
	const unsigned int publisher_timer_timeout = 1000;
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
	executor = rclc_executor_get_zero_initialized_executor();
	unsigned int num_handles = 1 + 0; //n_timers + n_subscriptions;
	RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &publisher_timer));
	
	while(true) {
		read_imu();
		rclc_executor_spin_some(&executor, 1000); // nanosec
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&imu_publisher, &node))
	RCCHECK(rcl_publisher_fini(&mag_publisher, &node))
	RCCHECK(rcl_publisher_fini(&tick_publisher, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}
