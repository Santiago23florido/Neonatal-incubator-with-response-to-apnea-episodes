#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cstdint>
#include <vector>

// Define motor driver pins
#define PIN_MOTOR 9          // Pin PWM para control de velocidad (Enable A)
#define PIN_MOTOR_IN2 10     // Pin de control de dirección 2 (IN2)
#define PIN_MOTOR_IN1 11     // Pin de control de dirección 1 (IN1)
#define PIN_STBY 12          // Pin de standby para el driver del motor (si aplica)

// Pin del LED integrado
#define PIN_LED 25           // Pin GPIO conectado al LED integrado

// Pines GPIO para el encoder
#define PIN_ENCODER_A 2      // Señal A del encoder
#define PIN_ENCODER_B 3      // Señal B del encoder

// Pin de entrada para controlar la velocidad
#define BUTTON_PIN 4        // Pin GP4 para control de velocidad

// Parámetros para el debouncing
#define DEBOUNCE_DELAY_MS 50  // Tiempo de debounce en milisegundos
#define BUTTON_SAMPLES_REQUIRED 3  // Número de muestras consecutivas para confirmar un cambio de estado

// Estructura para almacenar datos de tiempo y RPM
struct DataPoint {
    uint32_t time;
    float rpm;
};

class Motor {
private:
    // Pines del driver del motor
    const uint m_LED_PIN;
    const uint m_ENA_PIN;    // Pin PWM para control de velocidad (Enable A)
    const uint m_IN1_PIN;    // Pin de control de dirección 1 (IN1)
    const uint m_IN2_PIN;    // Pin de control de dirección 2 (IN2)
    const uint m_STBY_PIN;   // Pin de standby

    // Pines del encoder
    const uint m_ENCODER_A_PIN;
    const uint m_ENCODER_B_PIN;

    // Especificaciones del encoder
    const int m_TICKS_PER_REV;
    const float m_GR;

    // Slice de PWM
    uint pwmSlice;

    // Puntero a la instancia estática del motor actual
    static Motor* instance;

    // Variables estáticas para calcular las RPM
    static volatile int32_t encoder_ticks; // Almacena el número de ticks del encoder
    static int32_t last_ticks;
    static uint32_t last_measurement_time; // Tiempo de la última medición

    // Variables del filtro de paso bajo
    static constexpr float alpha = 0.15f;  // Factor de suavizado
    static float filtered_rpm;             // Valor de RPM filtrado anterior

    // Variables del controlador PID
    float setpoint;
    float y;
    float e;
    float y_prev;
    float u;
    float u_prev;
    float u_prev2;
    float e_prev;
    float e_prev2;
    float u_prev3;
    float u_prev4;
    float u_prev5;
    float last_u_new;  // Nueva variable para almacenar el último valor de u_new

    // Handler de interrupción del encoder privado
    static void encoder_irq_handler(uint gpio, uint32_t events) {
        // Envia la interrupción al handler de la instancia
        if (instance) {
            instance->handle_encoder_interrupt(gpio, events);
        }
    }

    // Método no estático para manejar la interrupción del encoder
    void handle_encoder_interrupt(uint gpio, uint32_t events) {
        bool encoder_a = gpio_get(m_ENCODER_A_PIN);
        bool encoder_b = gpio_get(m_ENCODER_B_PIN);

        // Determina la dirección basándose en ambos pines del encoder
        if (gpio == m_ENCODER_A_PIN) {
            if (encoder_a == encoder_b) {
                encoder_ticks--;        // Dirección inversa
            } else {
                encoder_ticks++;        // Dirección directa
            }
        } else if (gpio == m_ENCODER_B_PIN) {
            if (encoder_a == encoder_b) {
                encoder_ticks++;        // Dirección directa
            } else {
                encoder_ticks--;        // Dirección inversa
            }
        }
    }

    // Función para calcular la salida del PID
    float calculate_pid(float current_rpm, float dt) {
        // Si el setpoint es cero o casi cero, detenemos el motor directamente
        if (fabs(setpoint) < 0.1f) {
            // Reiniciamos las variables del PID para evitar acumulación de errores
            y_prev = 0.0f;
            e_prev = 0.0f;
            e_prev2 = 0.0f;
            u_prev = 0.0f;
            u_prev2 = 0.0f;
            u_prev3 = 0.0f;
            u_prev4 = 0.0f;
            u_prev5 = 0.0f;
            last_u_new = 0.0f;
            return 0.0f; // Retorna cero para detener el motor
        }
        
        y = current_rpm;
        float s = 16.185931122667185f * y -14.800320739556195f * y_prev;
        float t_sig = 1.38561038311099f * setpoint;
        e = t_sig - s;
        //e = setpoint - y;
        float u_new = 0.50f*e - 0.008435197091680f * u_prev -  0.007760381324340f * u_prev2 - 0.007139550818390f * u_prev3 + 0.525957047233630f * u_prev4 + 0.513378082000780f * u_prev5;
        //float u_new = u_prev + 0.150*e - 0.020*e_prev;
        
        // Aplicar zona muerta en la salida para evitar oscilaciones pequeñas
        if (fabs(u_new) < 3.0f) {
            u_new = 0.0f;
        }
        
        // Limitar la salida máxima
        if (u_new > 70.0f) {
            u_new = 70.0f;
        } else if (u_new < -70.0f) {
            u_new = -70.0f;
        }
        
        y_prev = y;
        e_prev = e;
        e_prev2=e_prev;
        u_prev5 = u_prev4;
        u_prev4 = u_prev3;
        u_prev3 = u_prev2;
        u_prev2 = u_prev;
        u_prev = u_new;
        
        // Guardamos el valor de u_new para poder acceder a él después
        last_u_new = u_new;
        
        return u_new;
    }

public:
    // Constructor
    Motor(uint led_pin, uint ena_pin, uint in1_pin, uint in2_pin, uint stby_pin,
          uint enc_a_pin, uint enc_b_pin, int ticks_per_rev = 64, float gear_ratio = 125.0f) :
            m_LED_PIN(led_pin), m_ENA_PIN(ena_pin), m_IN1_PIN(in1_pin), m_IN2_PIN(in2_pin), m_STBY_PIN(stby_pin),
            m_ENCODER_A_PIN(enc_a_pin), m_ENCODER_B_PIN(enc_b_pin),
            m_TICKS_PER_REV(ticks_per_rev), m_GR(gear_ratio),
            setpoint(0.0f), y_prev(0.0f), u(0.0f),e_prev(0.0f),e_prev2(0.0f),u_prev(0.0f), u_prev2(0.0f), u_prev3(0.0f), u_prev4(0.0f), u_prev5(0.0f), last_u_new(0.0f) {

        // Establece el puntero de instancia estático a este objeto
        instance = this;

        // Inicializa los pines GPIO
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
        gpio_put(m_STBY_PIN, 1); // Habilita el driver del motor por defecto

        // Configura el PWM para el control de velocidad en ENA_PIN
        gpio_set_function(m_ENA_PIN, GPIO_FUNC_PWM);
        pwmSlice = pwm_gpio_to_slice_num(m_ENA_PIN);
        pwm_set_wrap(pwmSlice, 65535);
        pwm_set_chan_level(pwmSlice, pwm_gpio_to_channel(m_ENA_PIN), 0);
        pwm_set_enabled(pwmSlice, true);

        // Configura los pines del encoder e interrupciones
        gpio_init(m_ENCODER_A_PIN);
        gpio_init(m_ENCODER_B_PIN);
        gpio_set_dir(m_ENCODER_A_PIN, GPIO_IN);
        gpio_set_dir(m_ENCODER_B_PIN, GPIO_IN);
        gpio_pull_up(m_ENCODER_A_PIN);
        gpio_pull_up(m_ENCODER_B_PIN);

        // Habilita las interrupciones para los pines A y B del encoder
        gpio_set_irq_enabled_with_callback(m_ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
        gpio_set_irq_enabled(m_ENCODER_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
        
        // Inicializar el tiempo de la primera medición
        last_measurement_time = to_ms_since_boot(get_absolute_time());
    }

    // Función para controlar la velocidad y dirección del motor DC
    void set_motor(float speed) {
        // Limita la velocidad entre -100 y 100
        if (speed > 100.0f) speed = 100.0f;
        if (speed < -100.0f) speed = -100.0f;

        // Calcula el valor PWM absoluto basado en el porcentaje de velocidad
        uint16_t pwm_value = static_cast<uint16_t>(fabs(speed) * 65535 / 100);

        if (speed > 0) {
            // Dirección directa
            gpio_put(m_IN1_PIN, 1);
            gpio_put(m_IN2_PIN, 0);
        } else if (speed < 0) {
            // Dirección inversa
            gpio_put(m_IN1_PIN, 0);
            gpio_put(m_IN2_PIN, 1);
        } else {
            // Detiene el motor
            gpio_put(m_IN1_PIN, 0);
            gpio_put(m_IN2_PIN, 0);
        }

        // Establece el ciclo de trabajo PWM directamente con el valor calculado
        pwm_set_gpio_level(m_ENA_PIN, pwm_value);
    }

    // Función para detener el motor
    void stop() {
        gpio_put(m_IN1_PIN, 0);
        gpio_put(m_IN2_PIN, 0);
        pwm_set_gpio_level(m_ENA_PIN, 0);
    }

    // Función para calcular las RPM del motor basándose en los ticks del encoder y aplicar un filtro de paso bajo
    float calculate_rpm(float *revs) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        float elapsed_sec = (current_time - last_measurement_time) / 1000.0f;  // Convertir ms a segundos
        
        int32_t ticks_since_last = encoder_ticks - last_ticks;
        last_ticks = encoder_ticks;
        
        if (revs != nullptr) {
            *revs = static_cast<float>(encoder_ticks) / m_TICKS_PER_REV / m_GR;
        }
        
        // Prevenir división por cero o valores irreales
        if (elapsed_sec < 0.001f) {
            elapsed_sec = 0.001f;  // Valor mínimo para prevenir valores extremos
        }
        
        float raw_rpm = (static_cast<float>(ticks_since_last) / m_TICKS_PER_REV) * (60.0f / elapsed_sec) / m_GR;
        
        // Actualiza el tiempo de la última medición
        last_measurement_time = current_time;
        
        return applyLowPassFilter(raw_rpm);
    }

    // Función para aplicar un filtro de paso bajo digital con umbral de ruido
    float applyLowPassFilter(float raw_rpm) {
        // Aplicamos primero un umbral para valores muy pequeños (ruido)
        if (fabs(raw_rpm) < 0.5f) {
            raw_rpm = 0.0f;
        }
        
        // Filtro de paso bajo con factor de suavizado
        filtered_rpm = alpha * raw_rpm + (1.0f - alpha) * filtered_rpm;
        
        // Aplicar segundo umbral para eliminar valores residuales muy pequeños
        if (fabs(filtered_rpm) < 0.2f) {
            filtered_rpm = 0.0f;
        }
        
        return filtered_rpm;
    }

    // Función para establecer las RPM objetivo para el controlador PID
    void set_motor_speed(float target_rpm) {
        printf("Estableciendo velocidad objetivo a %.2f RPM\n", target_rpm);
        setpoint = target_rpm;
    }

    // Función para actualizar la velocidad del motor usando control PID
    void update_motor_speed() {
        // Si el setpoint es exactamente cero, detenemos el motor directamente
        if (fabs(setpoint) < 0.1f) {
            stop(); // Detener el motor completamente
            return;
        }
        
        float revolutions = 0.0f;
        float current_rpm = calculate_rpm(&revolutions); // Obtiene las RPM actuales
        
        // Calcula el tiempo transcurrido desde la última actualización
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        float dt = (current_time - last_measurement_time) / 1000.0f; // Tiempo transcurrido en segundos
        
        // Asegúrate de que dt sea positivo y razonable
        if (dt < 0.001f) {
            dt = 0.001f;
        }
        
        float pid_output = calculate_pid(current_rpm, dt); // Calcula la salida del PID con dt dinámico

        // Limita la salida del PID a un rango razonable (ej., -100 a 100)
        float constrained_output = pid_output;
        if (constrained_output > 100.0f) constrained_output = 100.0f;
        if (constrained_output < -100.0f) constrained_output = -100.0f;
        
        // Si la salida es muy pequeña, consideramos que debe ser cero
        if (fabs(constrained_output) < 2.0f) {
            constrained_output = 0.0f;
        }
        
        set_motor(constrained_output);
    }

    // Obtiene el conteo de ticks del encoder actual
    int32_t getEncoderTicks() const {
        return encoder_ticks;
    }

    // Resetea los ticks del encoder
    void resetEncoderTicks() {
        encoder_ticks = 0;
        last_ticks = 0;
        filtered_rpm = 0.0f;
        last_measurement_time = to_ms_since_boot(get_absolute_time());
        e_prev = 0.0f;
        e_prev2=0.0f;
        y_prev = 0.0f;
        u_prev = 0.0f;
        u_prev2 = 0.0f;
        u_prev3 = 0.0f;
        u_prev4 = 0.0f;
        u_prev5 = 0.0f;
        last_u_new = 0.0f;
    }

    // Enciende/apaga el LED integrado para indicación
    void toggleLED() {
        gpio_put(m_LED_PIN, !gpio_get(m_LED_PIN));
    }
    
    // Función para obtener el último valor de u_new (salida del PID)
    float get_last_pid_output() const {
        return last_u_new;
    }
    
    // Función para obtener el tiempo transcurrido desde la última medición
    float get_elapsed_time() const {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        return (current_time - last_measurement_time) / 1000.0f;  // Convertir ms a segundos
    }
};

// Inicializa el puntero de instancia estático
Motor* Motor::instance = nullptr;

// Inicializa las variables estáticas
volatile int32_t Motor::encoder_ticks = 0;
int32_t Motor::last_ticks = 0;
float Motor::filtered_rpm = 0.0f;
uint32_t Motor::last_measurement_time = 0;

// Estructura para manejar el debouncing del botón
struct ButtonDebouncer {
    bool lastStableState;              // Estado estable previo del botón
    uint32_t lastDebounceTime;         // Tiempo de la última detección de cambio
    uint8_t stableCount;               // Contador para confirmar estado estable
    uint8_t consecutiveReadings[8];    // Historial de las últimas lecturas
    uint8_t readingIndex;              // Índice para el arreglo circular
    
    ButtonDebouncer() : lastStableState(false), lastDebounceTime(0), stableCount(0), readingIndex(0) {
        // Inicializar el historial de lecturas
        for(int i = 0; i < 8; i++) {
            consecutiveReadings[i] = 0;
        }
    }
    
    // Función que procesa el estado actual del botón y aplica debouncing avanzado
    bool update(bool currentReading) {
        uint32_t currentTime = to_ms_since_boot(get_absolute_time());
        bool stateChanged = false;
        
        // Guardamos la lectura actual en el historial circular
        consecutiveReadings[readingIndex] = currentReading ? 1 : 0;
        readingIndex = (readingIndex + 1) % 8;
        
        // Contamos cuántas lecturas iguales tenemos
        uint8_t sum = 0;
        for(int i = 0; i < 8; i++) {
            sum += consecutiveReadings[i];
        }
        
        // Determinamos el estado mayoritario (0 o 1)
        bool majorityState;
        if (sum >= 6) {  // Si al menos 6 de 8 lecturas son 1, consideramos el estado como 1
            majorityState = true;
        } else if (sum <= 2) {  // Si al menos 6 de 8 lecturas son 0, consideramos el estado como 0
            majorityState = false;
        } else {
            // Si hay dudas, mantenemos el estado anterior
            majorityState = lastStableState;
        }
        
        // Si ha pasado suficiente tiempo desde el último cambio de estado detectado
        if ((currentTime - lastDebounceTime) >= DEBOUNCE_DELAY_MS) {
            // Si el estado mayoritario es diferente al último estado estable
            if (majorityState != lastStableState) {
                stableCount++;
                
                // Si tenemos suficientes lecturas consistentes para confirmar un cambio real
                if (stableCount >= BUTTON_SAMPLES_REQUIRED) {
                    lastStableState = majorityState;
                    stateChanged = true;
                    stableCount = 0;
                    lastDebounceTime = currentTime;
                }
            } else {
                // Resetear el contador si volvemos al estado estable
                stableCount = 0;
            }
        }
        
        return stateChanged;
    }
    
    // Obtener el estado actual debounced
    bool getState() const {
        return lastStableState;
    }
};

int main() {
    // Inicializa stdio para la comunicación serial
    stdio_init_all();
    printf("Programa de identificación del sistema del motor con indicador LED...\n");

    // Inicializa el LED integrado como salida
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0); // Inicialmente apagado
    
    // Inicializa el pin del botón con resistencia de pull-down más fuerte y filtro de glitch
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_down(BUTTON_PIN);
    // Configurar filtro de glitch para el pin (si está disponible en el hardware)
    gpio_set_input_hysteresis_enabled(BUTTON_PIN, true);
    gpio_set_slew_rate(BUTTON_PIN, GPIO_SLEW_RATE_SLOW);
    
    // Crea un objeto para el debouncing del botón
    ButtonDebouncer buttonDebouncer;

    // Espera a que se establezca la conexión USB
    sleep_ms(10000);
    printf("Iniciando programa de control de motor...\n");

    // Crea un objeto motor con los pines especificados
    Motor motor(PIN_LED, PIN_MOTOR, PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_STBY, PIN_ENCODER_A, PIN_ENCODER_B);
    motor.resetEncoderTicks();

    const uint32_t sample_interval_ms = 1000;  // Intervalo para imprimir datos (1000ms)
    const uint32_t data_collection_interval_ms = 100;  // Intervalo para recolección de datos (100ms)
    const uint32_t pid_interval_ms = 10;       // Intervalo de muestreo para el PID (10ms)
    const uint32_t button_read_interval_ms = 5;  // Intervalo para leer el botón (5ms)
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_print_time = start_time;
    uint32_t last_data_collection_time = start_time;
    uint32_t last_pid_time = start_time;
    uint32_t last_button_time = start_time;
    bool led_state = false;

    // Vector para almacenar los datos de tiempo y RPM
    std::vector<DataPoint> data_points;
    // Reservar espacio para evitar realocaciones
    data_points.reserve(10);  // Reserva espacio para 10 puntos de datos (1 segundo / 100ms)

    // Agrega inicialización del PID
    float target_rpm = 0.0f; // RPM objetivo
    motor.set_motor_speed(target_rpm);

    printf("Sistema iniciado - PID controlará el motor para alcanzar %.1f RPM\n", target_rpm);

    while (true) { // Ciclo infinito
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Lectura y procesamiento del botón con debounce cada 5ms
        if (current_time - last_button_time >= button_read_interval_ms) {
            bool currentButtonState = gpio_get(BUTTON_PIN);
            bool stateChanged = buttonDebouncer.update(currentButtonState);
            
            // Si el estado del botón ha cambiado
            if (stateChanged) {
                bool stableState = buttonDebouncer.getState();
                if (stableState) {
                    target_rpm = 20.0f;
                    printf("Botón presionado: Setpoint cambiado a %.1f RPM\n", target_rpm);
                    motor.set_motor_speed(target_rpm);
                } else {
                    target_rpm = 0.0f;
                    printf("Botón liberado: Setpoint cambiado a %.1f RPM\n", target_rpm);
                    motor.set_motor_speed(target_rpm);
                }
            }
            
            last_button_time = current_time;
        }
        
        // Actualiza el PID cada 10 ms
        if (current_time - last_pid_time >= pid_interval_ms) {
            motor.update_motor_speed(); // Llama a la función de actualización del PID
            last_pid_time = current_time;
        }

        // Recolecta datos cada 100 ms y los almacena en el vector
        if (current_time - last_data_collection_time >= data_collection_interval_ms) {
            float revolutions = 0.0f;
            float rpm = motor.calculate_rpm(&revolutions);
            
            // Almacena los datos de tiempo y RPM en el vector
            DataPoint point;
            point.time = current_time - start_time;
            point.rpm = rpm;
            data_points.push_back(point);
            
            last_data_collection_time = current_time;
        }

        // Imprime el vector de datos cada 1000 ms (1 segundo) y cambia el LED
        if (current_time - last_print_time >= sample_interval_ms) {
            // Imprime el vector de datos (t, rpm) recolectado
            printf("Vector de datos acumulados (ms, RPM):");
            for (const auto& point : data_points) {
                printf(" %lu,%.2f", point.time, point.rpm);
            }
            printf("\n");
            
            // Limpia el vector para el próximo ciclo de recolección
            data_points.clear();
            
            // Cambia el estado del LED
            gpio_put(PIN_LED, led_state);
            led_state = !led_state;
            
            last_print_time = current_time;
        }

        sleep_ms(1); // Pequeño retardo para no saturar la CPU
    }

    return 0;
}