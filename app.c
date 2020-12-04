#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <maila_msgs/msg/esp32_data.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include <esp_log.h>
#endif

#include "mpu9250.c"

//#include <spi.h>     // http://freertoshal.github.io/doxygen/group__SPI.html
//#include <mpu9250.h> // http://freertoshal.github.io/doxygen/group__MPU9250.html

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;

maila_msgs__msg__Esp32Data mailamsg;

#define ENCODERS 5
int16_t prev_ticks[ENCODERS] = {};
int16_t ticks;

#define IMU_N_DATA 3

TickType_t TimePast;



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
	mailamsg.stamp.sec = last_call_time / 1000000;
	mailamsg.stamp.nanosec = last_call_time - (mailamsg.stamp.sec*1000000);

	// prepare mailamsg.int_data
	mailamsg.int_data.data = (int16_t *)calloc(ENCODERS, sizeof(int16_t));
	mailamsg.int_data.capacity = ENCODERS;
	mailamsg.int_data.size = ENCODERS;

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
	mailamsg.int_data.data[4] = 0;


	//updateIMU(TimePast);

	// prepare mailamsg.float_data
	mailamsg.float_data.data = (float *)calloc(IMU_N_DATA, sizeof(float));
	mailamsg.float_data.capacity = IMU_N_DATA;
	mailamsg.float_data.size = IMU_N_DATA;

	mailamsg.float_data.data[1] = _ax;
	mailamsg.float_data.data[1] = _ay;
	mailamsg.float_data.data[1] = _az;
	

	// send msg
	RCSOFTCHECK(rcl_publish(&publisher, &mailamsg, NULL));
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

	//RCLC_UNUSED(last_call_time);
	if (timer == NULL) {
		return;
	}

	
		

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

	// create publisher_timer
	rcl_timer_t publisher_timer;
	const unsigned int publisher_timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&publisher_timer,
		&support,
		RCL_MS_TO_NS(publisher_timer_timeout),
		publisher_timer_callback));

	/*
	// create imu_timer
	rcl_timer_t imu_timer;
	const unsigned int imu_timer_timeout = 500;
	RCCHECK(rclc_timer_init_default(
		&imu_timer,
		&support,
		RCL_MS_TO_NS(imu_timer_timeout),
		imu_timer_callback));
	*/

	// config pcnt
	setPCNTParams(GPIO_NUM_32,GPIO_NUM_33, PCNT_CHANNEL_0, PCNT_UNIT_0, 1); // encoder 0

	// config imu mpu9250
	setupIMU(18,19,23,5);
	
	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &publisher_timer));
	//RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		//usleep(100000);
	}

	// spin forever
	//rclc_executor_spin(&executor);

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}
