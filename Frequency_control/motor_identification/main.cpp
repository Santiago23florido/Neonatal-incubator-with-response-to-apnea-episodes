#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cstdint>
#include <vector>

// Define motor driver pins
#define PIN_MOTOR 9       // PWM pin for speed control (Enable A)
#define PIN_MOTOR_IN2 10    // Direction control pin 2 (IN2)
#define PIN_MOTOR_IN1 11    // Direction control pin 1 (IN1)
#define PIN_STBY 12     // Standby pin for the motor driver (if applicable)

// Onboard LED pin
#define PIN_LED 25      // GPIO pin connected to the onboard LED

// GPIO pins for encoder
#define PIN_ENCODER_A 2   // Encoder A signal
#define PIN_ENCODER_B 3   // Encoder B signal

class Motor {
private:
    // Motor driver pins
    const uint m_LED_PIN;
    const uint m_ENA_PIN;   // PWM pin for speed control (Enable A)
    const uint m_IN1_PIN;   // Direction control pin 1 (IN1)
    const uint m_IN2_PIN;   // Direction control pin 2 (IN2)
    const uint m_STBY_PIN;  // Standby pin

    // Encoder pins
    const uint m_ENCODER_A_PIN;
    const uint m_ENCODER_B_PIN;

    // Encoder specifications
    const int m_TICKS_PER_REV;
    const float m_GR;

    // PWM slice
    uint pwmSlice;

    // Static instance pointer for the current motor
    static Motor* instance;

    // Static variables for calculating RPM
    static volatile int32_t encoder_ticks; // Store the number of encoder ticks
    static int32_t last_ticks;
    static constexpr float time_interval_sec = 0.01f;   // Timer interval en segundos (10ms)

    // Low-pass filter variables
    static constexpr float alpha = 0.15f;   // Smoothing factor
    static float filtered_rpm;             // Previous filtered RPM value

    // Private encoder interrupt handler
    static void encoder_irq_handler(uint gpio, uint32_t events) {
        // Forward the interrupt to the instance's handler
        if (instance) {
            instance->handle_encoder_interrupt(gpio, events);
        }
    }

    // Non-static method to handle the encoder interrupt
    void handle_encoder_interrupt(uint gpio, uint32_t events) {
        bool encoder_a = gpio_get(m_ENCODER_A_PIN);
        bool encoder_b = gpio_get(m_ENCODER_B_PIN);

        // Determine direction based on both encoder pins
        if (gpio == m_ENCODER_A_PIN) {
            if (encoder_a == encoder_b) {
                encoder_ticks--;     // Reverse direction
            } else {
                encoder_ticks++;     // Forward direction
            }
        } else if (gpio == m_ENCODER_B_PIN) {
            if (encoder_a == encoder_b) {
                encoder_ticks++;     // Forward direction
            } else {
                encoder_ticks--;     // Reverse direction
            }
        }
    }

public:
    // Constructor
    Motor(uint led_pin, uint ena_pin, uint in1_pin, uint in2_pin, uint stby_pin,
          uint enc_a_pin, uint enc_b_pin, int ticks_per_rev = 64, float gear_ratio = 125.0f) :
          m_LED_PIN(led_pin), m_ENA_PIN(ena_pin), m_IN1_PIN(in1_pin), m_IN2_PIN(in2_pin), m_STBY_PIN(stby_pin),
          m_ENCODER_A_PIN(enc_a_pin), m_ENCODER_B_PIN(enc_b_pin),
          m_TICKS_PER_REV(ticks_per_rev), m_GR(gear_ratio) {

        // Set the static instance pointer to this object
        instance = this;

        // Initialize GPIO pins
        gpio_init(m_LED_PIN);
        gpio_set_dir(m_LED_PIN, GPIO_OUT);
        gpio_put(m_LED_PIN, 0); // Inicialmente apagado

        gpio_init(m_IN1_PIN);
        gpio_set_dir(m_IN1_PIN, GPIO_OUT);
        gpio_init(m_IN2_PIN);
        gpio_set_dir(m_IN2_PIN, GPIO_OUT);
        gpio_put(m_IN1_PIN, 0);
        gpio_put(m_IN2_PIN, 0);

        gpio_init(m_STBY_PIN);
        gpio_set_dir(m_STBY_PIN, GPIO_OUT);
        gpio_put(m_STBY_PIN, 1); // Enable the motor driver by default

        // Set up PWM for speed control on ENA_PIN
        gpio_set_function(m_ENA_PIN, GPIO_FUNC_PWM);
        pwmSlice = pwm_gpio_to_slice_num(m_ENA_PIN);
        pwm_set_wrap(pwmSlice, 65535);
        pwm_set_chan_level(pwmSlice, pwm_gpio_to_channel(m_ENA_PIN), 0);
        pwm_set_enabled(pwmSlice, true);

        // Configure encoder pins and interrupts
        gpio_init(m_ENCODER_A_PIN);
        gpio_init(m_ENCODER_B_PIN);
        gpio_set_dir(m_ENCODER_A_PIN, GPIO_IN);
        gpio_set_dir(m_ENCODER_B_PIN, GPIO_IN);
        gpio_pull_up(m_ENCODER_A_PIN);
        gpio_pull_up(m_ENCODER_B_PIN);

        // Enable interrupts for encoder pins A and B
        gpio_set_irq_enabled_with_callback(m_ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
        gpio_set_irq_enabled(m_ENCODER_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }

    // Function to control the DC motor speed and direction
    void set_motor(float speed) {
        // Clamp speed between -100 and 100
        if (speed > 100.0f) speed = 100.0f;
        if (speed < -100.0f) speed = -100.0f;

        // Calculate the absolute PWM value based on speed percentage
        uint16_t pwm_value = static_cast<uint16_t>(fabs(speed) * 65535 / 100);

        if (speed > 0) {
            // Forward direction
            gpio_put(m_IN1_PIN, 1);
            gpio_put(m_IN2_PIN, 0);
        } else if (speed < 0) {
            // Reverse direction
            gpio_put(m_IN1_PIN, 0);
            gpio_put(m_IN2_PIN, 1);
        } else {
            // Stop the motor
            gpio_put(m_IN1_PIN, 0);
            gpio_put(m_IN2_PIN, 0);
        }

        // Set PWM duty cycle directly with calculated value
        pwm_set_gpio_level(m_ENA_PIN, pwm_value);
    }

    // Function to stop the motor
    void stop() {
        gpio_put(m_IN1_PIN, 0);
        gpio_put(m_IN2_PIN, 0);
        pwm_set_gpio_level(m_ENA_PIN, 0);
    }

    // Function to calculate motor RPM based on encoder ticks and apply a low-pass filter
    float calculate_rpm(float *revs) {
        int32_t ticks_since_last = encoder_ticks - last_ticks;
        last_ticks = encoder_ticks;

        *revs = static_cast<float>(encoder_ticks) / m_TICKS_PER_REV / m_GR;
        float raw_rpm = (static_cast<float>(ticks_since_last) / m_TICKS_PER_REV) * (60.0f / time_interval_sec) / m_GR;
        return applyLowPassFilter(raw_rpm);
    }

    // Function to apply a digital low-pass filter
    float applyLowPassFilter(float raw_rpm) {
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
        filtered_rpm = 0.0f;
    }

    // Toggle the onboard LED for indication
    void toggleLED() {
        gpio_put(m_LED_PIN, !gpio_get(m_LED_PIN));
    }
};

// Initialize the static instance pointer
Motor* Motor::instance = nullptr;

// Initialize static variables
volatile int32_t Motor::encoder_ticks = 0;
int32_t Motor::last_ticks = 0;
float Motor::filtered_rpm = 0.0f;

int main() {
    // Initialize stdio for serial communication
    stdio_init_all();
    printf("Programa de identificación del sistema del motor con indicador LED...\n");

    // Initialize the onboard LED as output
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0); // Inicialmente apagado

    // Wait for USB connection to be established
    sleep_ms(2000);

    // Create a motor object with the specified pins
    Motor motor(PIN_LED, PIN_MOTOR, PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_STBY, PIN_ENCODER_A, PIN_ENCODER_B);
    motor.resetEncoderTicks();

    const int test_duration_ms = 5000; // Duración de la prueba en milisegundos
    const uint32_t sample_interval_ms = 10; // Intervalo de muestreo en milisegundos (¡más frecuente!)
    std::vector<uint32_t> time_data;
    std::vector<float> rpm_data;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_sample_time = start_time;
    const float test_speed = 30.0f; // Velocidad constante para la prueba (ajusta según necesidad)
    bool led_state = false;
    const uint32_t led_toggle_interval_ms = 100; // Intervalo para toggling del LED (ajustable)
    uint32_t last_led_toggle_time = start_time;

    printf("Aplicando un escalón de velocidad al %.1f%%\n", test_speed);
    motor.set_motor(test_speed);

    while (to_ms_since_boot(get_absolute_time()) - start_time < test_duration_ms) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_sample_time >= sample_interval_ms) {
            float revolutions = 0.0f;
            float rpm = motor.calculate_rpm(&revolutions);
            time_data.push_back(current_time - start_time);
            rpm_data.push_back(rpm);
            last_sample_time = current_time;
        }

        // Toggle el LED a una frecuencia menor para que sea visible
        if (current_time - last_led_toggle_time >= led_toggle_interval_ms) {
            gpio_put(PIN_LED, led_state);
            led_state = !led_state;
            last_led_toggle_time = current_time;
        }

        sleep_ms(1); // Pequeño retardo para no saturar la CPU
    }

    motor.stop();
    gpio_put(PIN_LED, 0); // Apagar el LED al finalizar la prueba
    printf("Prueba finalizada. Imprimiendo datos para identificación...\n");
    printf("Tiempo (ms),RPM\n"); // Encabezado CSV

    // Imprimir los datos almacenados en los vectores en formato CSV
    for (size_t i = 0; i < time_data.size(); ++i) {
        printf("%lu,%.2f\n", time_data[i], rpm_data[i]);
    }

    printf("Fin de la impresión de datos.\n");

    while (1) {
        sleep_ms(1000); // Mantener la Pico activa
    }

    return 0;
}