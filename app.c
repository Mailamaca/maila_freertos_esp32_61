#include <stdio.h>
#include <unistd.h>
//#include <vector> 

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
//#include <std_msgs/msg/int32.h>
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
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;

maila_msgs__msg__Esp32Data mailamsg;

#define ENCODERS 5
int16_t prev_ticks[ENCODERS] = {};



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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

	//RCLC_UNUSED(last_call_time);
	if (timer == NULL) {
		return;
	}
	
	// fill mailamsg.stamp
	mailamsg.stamp.sec = last_call_time / 1000;
	mailamsg.stamp.nanosec = last_call_time - mailamsg.stamp.sec;

	// calc encoders
	int16_t delta_ticks[ENCODERS] = {};
	int16_t value;

	// enc 0
	pcnt_get_counter_value(PCNT_UNIT_0, &value);
	delta_ticks.push_back(getPCNTDelta(prev_ticks[0], value));
	prev_ticks[0] = value;

	// fill mailamsg.int_data
	mailamsg.int_data.data = (int16_t *)calloc(ENCODERS, sizeof(int16_t));
	mailamsg.int_data.capacity = ENCODERS;
	mailamsg.int_data.size = ENCODERS;
	mailamsg.int_data.data = &delta_ticks;

	// send msg
	RCSOFTCHECK(rcl_publish(&publisher, &mailamsg, NULL));
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

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 500;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// config pcnt
	setPCNTParams(GPIO_NUM_32,GPIO_NUM_33, PCNT_CHANNEL_0, PCNT_UNIT_0, 1); // encoder 0

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	mailamsg.stamp.sec = 0;
	mailamsg.stamp.nanosec = 0;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		//usleep(100000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}
