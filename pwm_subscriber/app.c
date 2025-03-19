#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <my_custom_message/msg/pwm_message.h>

#include <driver/gpio.h>
#include <driver/ledc.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// Macro functions
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


// Constants
#define FRAME_TIME 100 // 1000 / FRAME_TIME = FPS
#define SLEEP_TIME 10

// PINS
#define LED_BUILTIN 33
#define LEFT_PWM_PIN 13
#define RIGHT_PWM_PIN 15
#define IN1 26
#define IN2 27
#define IN3 2
#define IN4 4

#define LEFT_PWM LEDC_CHANNEL_2
#define RIGHT_PWM LEDC_CHANNEL_3

// Other PWM settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE

my_custom_message__msg__PwmMessage msg;
rcl_publisher_t publisher;

// Function forward declarations
void setupPins();
void setupRos();
void empty_callback(const void *msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
float fmap(float val, float in_min, float in_max, float out_min, float out_max);
int cap_int(int value);

// Main
void appMain(void *arg) {
    setupPins();
    setupRos();
}

void setupPins() {

    // Led. Set it to GPIO_MODE_INPUT_OUTPUT, because we want to read back the state we set it to.
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);


    gpio_set_direction(IN1, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(IN2, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(IN3, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(IN4, GPIO_MODE_INPUT_OUTPUT);

    gpio_set_level(IN1, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN2, 1);
    gpio_set_level(IN4, 1);


    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Configure PWM channels and assign output pins
    ledc_channel_config_t ledc_channel[2] = {
        {
            .channel    = LEFT_PWM,
            .duty       = 0,
            .gpio_num   = LEFT_PWM_PIN,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = RIGHT_PWM,
            .duty       = 0,
            .gpio_num   = RIGHT_PWM_PIN,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
    };

    for (int i = 0; i < 2; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
}

void setupRos() {
    // Micro ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_pwm_sub", "", &support));

    // create subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(my_custom_message, msg, PwmMessage),
        "/pwm"));

    // create timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(FRAME_TIME),
        timer_callback));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &empty_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SLEEP_TIME));
        usleep(SLEEP_TIME * 1000);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

// We don't really need the callback, because msg is set anyway
void empty_callback(const void *msgin) {
}

// Each frame, check msg data and set PWM channels accordingly
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

    gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 1);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 1);
    
    if (timer == NULL) {
        return;
    }
    if (msg.pwm_left < 0){
        gpio_set_level(IN1, 1);
        gpio_set_level(IN2, 0);
    }
    else{
        gpio_set_level(IN1, 0);
        gpio_set_level(IN2, 1);
    }

    if (msg.pwm_right < 0){
        gpio_set_level(IN3, 1);
        gpio_set_level(IN4, 0);
    }
    else{
        gpio_set_level(IN3, 0);
        gpio_set_level(IN4, 1);
    }

    ledc_set_duty(PWM_MODE, LEFT_PWM, abs(msg.pwm_left));
    ledc_set_duty(PWM_MODE, RIGHT_PWM, abs(msg.pwm_right));

    ledc_update_duty(PWM_MODE, LEFT_PWM);
    ledc_update_duty(PWM_MODE, RIGHT_PWM);
}

/*
config && build firmware:
	ros2 run micro_ros_setup configure_firmware.sh pwm_subscriber -t udp -i <ip> -p <port> && ros2 run micro_ros_setup build_firmware.sh
flash && build agent:
	ros2 run micro_ros_setup flash_firmware.sh && ros2 run micro_ros_setup build_agent.sh
source && run agent:
	source install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port <port>
*/
int cap_int(int value){
    if (value > 4095){
        value = 4095;
    }
    if (value < 0){
        value = 0;
    }
    return value;
}
