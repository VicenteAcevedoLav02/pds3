#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include <esp_camera.h>
#include <tensorflow/lite/c/common.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include "model_1.h"  // Archivo generado con xxd

#define LED_BUILTIN 33
#define NUM_CLASSES 6
#define TENSOR_ARENA_SIZE 2000  // Ajusta según sea necesario

// Arreglo para el tensor de entrada/salida
uint8_t tensor_arena[TENSOR_ARENA_SIZE];
tflite::ErrorReporter error_reporter;

void blinkLED(int times, int delayTime) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delayTime);
        digitalWrite(LED_BUILTIN, LOW);
        delay(delayTime);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    // Configuración de la cámara
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 5;
    config.pin_d1 = 18;
    config.pin_d2 = 19;
    config.pin_d3 = 21;
    config.pin_d4 = 36;
    config.pin_d5 = 39;
    config.pin_d6 = 34;
    config.pin_d7 = 35;
    config.pin_xclk = 0;
    config.pin_pclk = 22;
    config.pin_vsync = 25;
    config.pin_href = 23;
    config.pin_sccb_sda = 26;
    config.pin_sccb_scl = 27;
    config.pin_pwdn = 32;
    config.pin_reset = -1;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_96X96;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    // Inicialización de la cámara
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Error: Camera init failed with error 0x%x\n", err);
        return;
    }

    // Cargar modelo y configurarlo en TensorFlow Lite
    static tflite::MicroMutableOpResolver<5> resolver;
    resolver.AddConv2D();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    
    tflite::MicroInterpreter interpreter(model_1, resolver, tensor_arena, sizeof(tensor_arena), &error_reporter);
    
    // Inicializa el modelo
    interpreter.AllocateTensors();

    // Ejecutar ciclo de clasificación
    while (1) {
        blinkLED(3, 300);  // Parpadea 3 veces

        digitalWrite(LED_BUILTIN, HIGH);
        delay(3000); // Esperar antes de tomar la imagen

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Error: No frame buffer");
            return;
        }

        // Procesamiento de la imagen
        // Aquí debes implementar el código para pasar la imagen a los tensores de entrada
        // y luego ejecutar la inferencia con el modelo.

        // Asegúrate de tener un tensor de entrada configurado
        float* input = interpreter.input(0)->data.f; // Asumiendo que el modelo espera un tensor de flotantes
        // Procesa fb->buf y almacena los valores en el tensor de entrada
        
        esp_camera_fb_return(fb);

        // Ejecuta el modelo
        interpreter.Invoke();

        // Obtén el resultado de la inferencia
        float* output = interpreter.output(0)->data.f;
        int result_class = 0;
        float max_value = output[0];
        for (int i = 1; i < NUM_CLASSES; i++) {
            if (output[i] > max_value) {
                max_value = output[i];
                result_class = i;
            }
        }

        Serial.printf("Clase detectada: %d con confianza: %.2f\n", result_class, max_value);
    }
}

void loop() {
    // El loop no se usa en este ejemplo
}
