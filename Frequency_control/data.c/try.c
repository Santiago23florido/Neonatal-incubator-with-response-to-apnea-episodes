#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <pico/stdlib.h>
#include <math.h>

#include "pico_uart_transports.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

class Motor {
private:
    // Motor driver pins (L298N)
  // Motor driver pins
    const uint LED_PIN;
    const uint ENA_PIN;  // PWM pin for speed control (Enable A)
    const uint IN1_PIN;  // Direction control pin 1 (IN1)
    const uint IN2_PIN;  // Direction control pin 2 (IN2)

    // Encoder pins
    const uint ENCODER_A_PIN;
    const uint ENCODER_B_PIN;

    // Encoder specifications
    const int TICKS_PER_REV;
    const float GR;

    // PWM slice and encoder tick counter
    uint pwmSlice;                 // PWM slice for motor control

     // Static instance pointer for the current motor
    static Motor* instance;

    // Static variables for calculating RPM
    static volatile int32_t encoder_ticks; // Store the number of encoder ticks
    static int32_t last_ticks;
    static constexpr float time_interval_sec = 0.1f;  // Timer interval in seconds (100ms)

    // Low-pass filter variables
    static constexpr float alpha = 0.15f;  // Smoothing factor
    static float filtered_rpm;              // Previous filtered RPM value

    // PID controller variables
    float Kp;
    float Ki;
    float Kd;

    float previous_error = 0.0f; // Previous error for the derivative term
    float integral = 0.0f;       // Accumulated integral error
    float dt = time_interval_sec; // Time interval for PID calculations (same as RPM calculation interval)


// Private encoder interrupt handler
    static void encoder_a_irq_handler(uint gpio, uint32_t events) {
        // Forward the interrupt to the instance's handler
        instance->handle_encoder_interrupt(gpio, events);
    }

    // Non-static method to handle the encoder interrupt
    void handle_encoder_interrupt(uint gpio, uint32_t events) {
        bool encoder_a = gpio_get(ENCODER_A_PIN);
        bool encoder_b = gpio_get(ENCODER_B_PIN);

        if (gpio == ENCODER_A_PIN) {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
                encoder_ticks++;  // Forward direction
            } else {
                encoder_ticks--;  // Reverse direction
            }
        } else if (gpio == ENCODER_B_PIN) {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
                encoder_ticks--;  // Forward direction
            } else {
                encoder_ticks++;  // Reverse direction
            }
        }
    }

public:
    // Constructor
     Motor(uint led_pin = 25, uint ena_pin = 2, uint in1_pin = 3, uint in2_pin = 4, 
          uint enc_a_pin = 5, uint enc_b_pin = 6, 
          int ticks_per_rev = 64, float gear_ratio = 50.0f, 
          float kp = 0.1158f, float ki = 0.4634f, float kd = 0.0f) :
          LED_PIN(led_pin), ENA_PIN(ena_pin), IN1_PIN(in1_pin), IN2_PIN(in2_pin),
          ENCODER_A_PIN(enc_a_pin), ENCODER_B_PIN(enc_b_pin),
          TICKS_PER_REV(ticks_per_rev), GR(gear_ratio), 
          Kp(kp), Ki(ki), Kd(kd) {

        // Set the static instance pointer to this object
        instance = this;

        // Configure GPIO pins for the motor driver
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        gpio_put(LED_PIN, 0);  // Turn off LED initially

        gpio_init(IN1_PIN);
        gpio_init(IN2_PIN);
        gpio_set_dir(IN1_PIN, GPIO_OUT);
        gpio_set_dir(IN2_PIN, GPIO_OUT);

        // Set up PWM for speed control on ENA_PIN
        gpio_set_function(ENA_PIN, GPIO_FUNC_PWM);
        pwmSlice = pwm_gpio_to_slice_num(ENA_PIN);
        pwm_set_wrap(pwmSlice, 65535);  // Set PWM wrap for 16-bit resolution
        pwm_set_chan_level(pwmSlice, PWM_CHAN_A, 0);  // Set initial duty cycle (stopped)
        pwm_set_enabled(pwmSlice, true);  // Enable PWM

        // Configure encoder pins and interrupts
        gpio_init(ENCODER_A_PIN);
        gpio_init(ENCODER_B_PIN);
        gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
        gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
        gpio_pull_up(ENCODER_A_PIN);
        gpio_pull_up(ENCODER_B_PIN);

        // Enable interrupts for encoder pins A and B
        gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_a_irq_handler);
        gpio_set_irq_enabled(ENCODER_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }

    // Function to control the DC motor speed and direction
    void set_motor(float speed) {
        uint16_t pwm_value = (uint16_t)fabs(speed);  // Take the absolute value of the speed

        if (speed > 0) {
            // Forward direction
            gpio_put(IN1_PIN, 1);
            gpio_put(IN2_PIN, 0);
        } else if (speed < 0) {
            // Reverse direction
            gpio_put(IN1_PIN, 0);
            gpio_put(IN2_PIN, 1);
        } else {
            // Stop the motor
            gpio_put(IN1_PIN, 0);
            gpio_put(IN2_PIN, 0);
        }

        // Set PWM duty cycle for speed control, scaled to 16-bit resolution
        pwm_set_gpio_level(ENA_PIN, (uint16_t)(pwm_value * 65535 / 100));  // Map 0-100% to 16-bit PWM
    }

    // Function to calculate motor RPM based on encoder ticks and apply a low-pass filter
    void calculate_rpm(float *revs, float *rpm ) {
        // Calculate ticks since the last calculation
		
        int32_t ticks_since_last = encoder_ticks - last_ticks;
        last_ticks = encoder_ticks;

		*revs = encoder_ticks / TICKS_PER_REV;

        // Calculate raw RPM
        float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / time_interval_sec) * (1.0f / GR);

        // Apply low-pass filter to the raw RPM value
        *rpm = applyLowPassFilter(raw_rpm);
    }

    // Function to apply a digital low-pass filter
    float applyLowPassFilter(float raw_rpm) {
        // First-order digital low-pass filter
        filtered_rpm = alpha * raw_rpm + (1.0f - alpha) * filtered_rpm;
        return filtered_rpm;
    }

    // Get the current encoder tick count
    int32_t getEncoderTicks() const {
        return encoder_ticks;
    }

    // Reset encoder ticks
    void resetEncoderTicks() {
        encoder_ticks = 0;
        last_ticks = 0;
        filtered_rpm = 0.0f;  // Reset filtered RPM value
        integral = 0.0f;      // Reset integral term
        previous_error = 0.0f; // Reset previous error
    }

    // Toggle the onboard LED for indication
    void toggleLED() {
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
    }
};

// Initialize the static instance pointer
Motor* Motor::instance = nullptr;

// Initialize static variables
volatile int32_t Motor::encoder_ticks = 0;
int32_t Motor::last_ticks = 0;
float Motor::filtered_rpm = 0.0f;

// Create a motor object with specific motor and encoder pins, and PID parameters
// Motor(uint led_pin = 25, uint ena_pin = 11, uint in1_pin = 13, uint in2_pin = 12, uint enc_a_pin = 26, uint enc_b_pin = 27, int ticks_per_rev = 64, float gear_ratio = 50.0f, float kp = 0.1158f, float ki = 0.4634f, float kd = 0.0f)

Motor motor(25, 11, 13, 12, 26, 27, 64, 50.0f, 0.1f, 0.1f, 0.01f);

float cmd = 0.0f; // cmd Motor


// Initialize global variables for ROS2
rcl_subscription_t cmd_subs; // Subscriber for speed setpoint
geometry_msgs_msg_Twist cmd_msg; // Message for speed setpoint

rcl_publisher_t debug_pub;    // Publisher for Debugging messages
geometry_msgs_msg_Twist debug_msg;     // Message type for the Debugging messages

// Function to map a value from one range to another
float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ROS2 subscriber callback for receiving Twist messages
void cmd_callback(const void * msgin) {
    const geometry_msgs_msgTwist * twist_msg_const = (const geometry_msgsmsg_Twist *)msgin;
    printf("Received speed setpoint: %f\n", twist_msg_const->angular.z);

    // Setpoint for the desired RPM
    cmd = (float)map(twist_msg_const->angular.z, -3.0, 3.0, -100.0, 100.0);

}

// Timer callback to calculate and publish RPM
void debug_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

    // Example usage: Calculate the current RPM
	float position = 0.0f;
	float speed = 0.0f;
    motor.calculate_rpm(&position, &speed);

    // Apply the control signal to the motor
    motor.set_motor(cmd);


    motor.toggleLED();  // Toggle the onboard LED

	// Publish the debug message
    debug_msg.linear.x = position;
    debug_msg.linear.y = speed;
	debug_msg.linear.z = cmd;

    rcl_ret_t ret2 = rcl_publish(&debug_pub, &debug_msg, NULL);
}



int main() {
    //stdio_init_all();  // Initialize standard I/O (for debugging)
    
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Initialize ROS2 components
    rcl_timer_t debug_timer;
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;

    rcl_ret_t ret = rmw_uros_ping_agent(1000, 120);  // Wait for the Micro-ROS agent
    if (ret != RCL_RET_OK) {
        printf("Failed to connect to Micro-ROS agent.\n");
        return ret;
    }

    // Initialize support and node
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);


    // Initialize the publisher to publish setpoint RPM
    rclc_publisher_init_default(
        &debug_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug_pub"
    );

    // Initialize the timer for RPM (100ms interval)
    rclc_timer_init_default(
        &debug_timer,
        &support,
        RCL_MS_TO_NS(100),
        debug_timer_callback
    );

    // Initialize the subscriber to receive Twist messages
    rclc_subscription_init_default(
        &cmd_subs,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd"
    );


    // Initialize executor and add the subscription
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    
    rclc_executor_add_subscription(&executor, &cmd_subs, &cmd_msg, &cmd_callback, ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &debug_timer);



    while (true) {

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    return 0;
}