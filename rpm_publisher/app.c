#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <my_custom_message/msg/my_custom_message.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "driver/timer.h"
#include "driver/gpio.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define LEFT_ENCODER_PIN 32
#define RIGHT_ENCODER_PIN 33

my_custom_message__msg__MyCustomMessage msg;
rcl_publisher_t publisher;

//encoder variables
bool old_state = 0, new_state = 0;
int steps=0, reading_time = 250000;
float rpm = 0;

//foward declarations
void setupPins();
void setupRos();
float rpm_measure(int encoder_pin);

void appMain(void *arg){
	setupPins();
	setupRos();
}
void setupPins(){
    gpio_set_direction(LEFT_ENCODER_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(RIGHT_ENCODER_PIN, GPIO_MODE_INPUT);
}
void setupRos(void * arg){
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	const char * node_name = "esp";
	RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

	// create left publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node, 
		ROSIDL_GET_MSG_TYPE_SUPPORT(my_custom_message, msg, MyCustomMessage), 
		"rpm"));

	//Init rpm message
	msg.rpm_left = 0;
	msg.rpm_right = 0;

	while(1){
		msg.rpm_left = rpm_measure(LEFT_ENCODER_PIN);
		msg.rpm_right = rpm_measure(RIGHT_ENCODER_PIN);
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		usleep(100000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))
  	vTaskDelete(NULL);
}
float rpm_measure(int encoder_pin){
	steps = 0;
    uint64_t init_time = esp_timer_get_time();
    while((esp_timer_get_time() - init_time) < reading_time){
        new_state = gpio_get_level(encoder_pin);
            if((new_state == 1) && (old_state == 0)){
                steps += 1;
            }
        old_state = new_state;
    }
    rpm = 12 * steps;
	return rpm;
}

/*
config && build firmware:
	ros2 run micro_ros_setup configure_firmware.sh rpm_publisher -t udp -i 192.168.1.102 -p 8888 && ros2 run micro_ros_setup build_firmware.sh
flash && build agent:
	ros2 run micro_ros_setup flash_firmware.sh && ros2 run micro_ros_setup build_agent.sh
source:
	source install/local_setup.bash
run agent:
	ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
*/