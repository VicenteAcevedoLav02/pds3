/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "model_settings.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "main_functions.h"
#define LED_GPIO GPIO_NUM_4 // Cambia este valor según el pin del LED en tu ESP32 CAM
static uint8_t s_led_state = 0;
#define configTICK_RATE_HZ 1000

#define SERVO_MIN_PULSEWIDTH 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180      // Maximum angle in degree up to which servo can rotate

// GPIO for the servos
#define SERVO_PIN1 12 
#define SERVO_PIN2 13


#include "detection_responder.h"
#include "image_provider.h"
#include "model_settings.h"
#include "person_detect_model_data.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_log.h>
#include "esp_main.h"

// Globals, used for compatibility with Arduino-style sketches.
namespace {
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;

// In order to use optimized tensorflow lite kernels, a signed int8_t quantized
// model is preferred over the legacy unsigned model format. This means that
// throughout this project, input images must be converted from unisgned to
// signed format. The easiest and quickest way to convert from unsigned to
// signed 8-bit integers is to subtract 128 from the unsigned value to get a
// signed value.

#ifdef CONFIG_IDF_TARGET_ESP32S3
  constexpr int scratchBuf Size = 40 * 1024;
#else
  constexpr int scratchBufSize = 30000;
#endif
  // An area of memory to use for input, output, and intermediate arrays.
  constexpr int kTensorArenaSize = 4 * 81 * 1024 + scratchBufSize;//96 * 96 * sizeof(uint) + 636904; // 4 * 81 * 1024 + scratchBufSize;
  static uint8_t *tensor_arena;                                     //[kTensorArenaSize]; // Maybe we should move this to external
} // namespace

void configure_led(void) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

// Parpadeo del LED
void blink_led(void) {
    gpio_set_level(LED_GPIO, s_led_state);
    s_led_state = !s_led_state;
}
// Function to initialize GPIO for the servos
static void mcpwm_example_gpio_initialize(void) {
    printf("Initializing MCPWM servo control GPIO...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PIN1); // Set GPIO 12 as PWM0A (Servo 1)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, SERVO_PIN2); // Set GPIO 13 as PWM0B (Servo 2)
}

// Function to control the servo angle based on identifier
void servo_control(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, mcpwm_operator_t op, float angle) {
    // Calculate pulse width (500us - 2500us) corresponding to the angle (-90° - 270°)
    uint32_t duty_us = (500 + ((angle + 90) / 360.0) * 2000);
    mcpwm_set_duty_in_us(mcpwm_num, timer_num, op, duty_us);
}

// Task to move each servo independently
static void servo_task(void *arg) {
    int servo_id = (int)arg;  // Get servo identifier (1 or 2)

    // Configure servo and operator based on identifier
    mcpwm_operator_t op = (servo_id == 1) ? MCPWM_OPR_A : MCPWM_OPR_B;

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;  // Frequency = 50Hz, i.e., 20ms period for servos
    pwm_config.cmpr_a = 0;      // Duty cycle of PWM0A = 0 (for servo 1)
    pwm_config.cmpr_b = 0;      // Duty cycle of PWM0B = 0 (for servo 2)
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); // Initialize PWM for both servos


    servo_control(MCPWM_UNIT_0, MCPWM_TIMER_0, op, 180);
    printf("Servo %d moved to 180 degrees\n", servo_id);

}


// The name of this function is important for Arduino compatibility.
void setup() {
  
  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_person_detect_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model provided is schema version %d not equal to supported "
                "version %d.", model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  if (tensor_arena == NULL) {
    tensor_arena = (uint8_t *) heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (tensor_arena == NULL) {
    printf("Couldn't allocate memory of %d bytes\n", kTensorArenaSize);
    return;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  //
  // tflite::AllOpsResolver resolver;
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroMutableOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddMaxPool2D();
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddReshape();
  micro_op_resolver.AddFullyConnected();
  micro_op_resolver.AddSoftmax();
  micro_op_resolver.AddQuantize();

  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  // Get information about the memory area to use for the model's input.
  input = interpreter->input(0);

#ifndef CLI_ONLY_INFERENCE
  // Initialize Camera
  TfLiteStatus init_status = InitCamera();
  if (init_status != kTfLiteOk) {
    MicroPrintf("InitCamera failed\n");
    return;
  }
#endif
}

#ifndef CLI_ONLY_INFERENCE
// The name of this function is important for Arduino compatibility.
void loop() {
    int lockerState = gpio_get_level(GPIO_NUM_4);
    gpio_reset_pin(GPIO_NUM_12);
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_12, 0);
    gpio_set_level(GPIO_NUM_12, 0);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
    int masterMode = gpio_get_level(GPIO_NUM_2);
    if (masterMode == 0) {
            MicroPrintf("Admin Mode Activated");
            while(true){
            mcpwm_example_gpio_initialize(); // Initialize servo GPIOs
             MicroPrintf("FOR SERVO GPIO 12");
             xTaskCreate(servo_task, "servo_task_1", 2048, (void *)1, 5, NULL); // Task for servo 1
             //xTaskCreate(servo_task, "servo_task_2", 2048, (void *)2, 5, NULL); // Task for servo 2
             
             vTaskDelay(600 / portTICK_PERIOD_MS); // Espera 1 segundo
             MicroPrintf("FOR SERVO GPIO 13");
             xTaskCreate(servo_task, "servo_task_2", 2048, (void *)2, 5, NULL); // Task for servo 1
            
             vTaskDelay(600 / portTICK_PERIOD_MS); // Espera 1 segundo
             }
             while (true) {
              vTaskDelay(portMAX_DELAY); // Espera indefinida para detener el loop
              }
        } else {
            MicroPrintf("User Mode");
        }
 

    if (lockerState == 1) {
        MicroPrintf("Locker 1");
    } else {
        MicroPrintf("Locker 0");
    }
  int Keys[2][4] = {{5, 5, 5, 5}, {0, 0, 0, 0}};
  int KeyCompare[4];
    // Configurar el LED
    configure_led();
    
    // Repetir cuatro veces el ciclo de toma de imagen
    for (int i = 0; i < 4; i++) {
        MicroPrintf("Photo in 3...");
        gpio_set_level(LED_GPIO, 1); // Parpadeo 1 - Encender LED
        vTaskDelay(300 / portTICK_PERIOD_MS); // Espera 1 segundo
        gpio_set_level(LED_GPIO, 0); // Apagar LED
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera adicional para parpadeo

        MicroPrintf("Photo in 2...");
        gpio_set_level(LED_GPIO, 1); // Parpadeo 2 - Encender LED
        vTaskDelay(300 / portTICK_PERIOD_MS); // Espera 1 segundo
        gpio_set_level(LED_GPIO, 0); // Apagar LED
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera adicional para parpadeo

        MicroPrintf("Photo in 1...");
        gpio_set_level(LED_GPIO, 1); // Encender LED por 1 segundo
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_GPIO, 0); // Apagar LED
        
        if (kTfLiteOk != GetImage(kNumCols, kNumRows, kNumChannels, input->data.int8)) {
            MicroPrintf("Image capture failed.");
            continue;
        }

        if (kTfLiteOk != interpreter->Invoke()) {
            MicroPrintf("Invoke failed.");
            continue;
        }

        TfLiteTensor* output = interpreter->output(0);

        // Obtener los puntajes de cada clase
        int scores[6] = {
            output->data.uint8[kUnDedoIndex],
            output->data.uint8[kRockIndex],
            output->data.uint8[kTresDedosIndex],
            output->data.uint8[kPulgarIndex],
            output->data.uint8[kAbiertaIndex],
            output->data.uint8[kCerradoIndex]
        };

        // Calcular el índice con mayor puntaje
        int maxIndex = 0;
        int maxScore = scores[0];
        for (int j = 1; j < 6; j++) {
            if (scores[j] > maxScore) {
                maxScore = scores[j];
                maxIndex = j;
            }
        }

        // Almacenar el índice en KeyCompare
        KeyCompare[i] = maxIndex;
        MicroPrintf("Clase con mayor puntuación: %d", maxIndex);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera antes de la siguiente iteración
;
    }
  

    // Imprimir KeyCompare
    MicroPrintf("KeyCompare: ");
    for (int i = 0; i < 4; i++) {
        MicroPrintf("%d ", KeyCompare[i]);
    }

    // Comparar KeyCompare con Keys[0] y Keys[1]
  bool isEqualKey0 = true;
  bool isEqualKey1 = true;

  for (int i = 0; i < 4; i++) {
      MicroPrintf("Índice %d introducido es: %d", i, KeyCompare[i]);
      if (KeyCompare[i] != Keys[0][i]) isEqualKey0 = false;
      if (KeyCompare[i] != Keys[1][i]) isEqualKey1 = false;
  }

  // Configurar GPIO12 y GPIO13 como salidas
  

  if (lockerState == 1) {
      MicroPrintf("Comparación con Keys[1]: %s", isEqualKey1 ? "Iguales" : "Diferentes");
      mcpwm_example_gpio_initialize(); // Initialize servo GPIOs
      if (isEqualKey1) {
          MicroPrintf("Activando GPIO13");
          xTaskCreate(servo_task, "servo_task_1", 2048, (void *)1, 5, NULL); // Task for servo 1
      }
  } else {
      MicroPrintf("Comparación con Keys[0]: %s", isEqualKey0 ? "Iguales" : "Diferentes");
      if (isEqualKey0) {
          MicroPrintf("Activando GPIO12");
          xTaskCreate(servo_task, "servo_task_2", 2048, (void *)2, 5, NULL); // Task for servo 2
      }
  }
   while (true) {
        vTaskDelay(portMAX_DELAY); // Espera indefinida para detener el loop
    }

}

#endif

#if defined(COLLECT_CPU_STATS)
  long long total_time = 0;
  long long start_time = 0;
  extern long long softmax_total_time;
  extern long long dc_total_time;
  extern long long conv_total_time;
  extern long long fc_total_time;
  extern long long pooling_total_time;
  extern long long add_total_time;
  extern long long mul_total_time;
#endif

void run_inference(void *ptr) {
  /* Convert from uint8 picture data to int8 */
  for (int i = 0; i < kNumCols * kNumRows; i++) {
    input->data.int8[i] = ((uint8_t *) ptr)[i] ^ 0x80;
  }

#if defined(COLLECT_CPU_STATS)
  long long start_time = esp_timer_get_time();
#endif
  // Run the model on this input and make sure it succeeds.
  if (kTfLiteOk != interpreter->Invoke()) {
    MicroPrintf("Invoke failed.");
  }

#if defined(COLLECT_CPU_STATS)
  long long total_time = (esp_timer_get_time() - start_time);
  printf("Total time = %lld\n", total_time / 1000);
  //printf("Softmax time = %lld\n", softmax_total_time / 1000);
  printf("FC time = %lld\n", fc_total_time / 1000);
  printf("DC time = %lld\n", dc_total_time / 1000);
  printf("conv time = %lld\n", conv_total_time / 1000);
  printf("Pooling time = %lld\n", pooling_total_time / 1000);
  printf("add time = %lld\n", add_total_time / 1000);
  printf("mul time = %lld\n", mul_total_time / 1000);

  /* Reset times */
  total_time = 0;
  //softmax_total_time = 0;
  dc_total_time = 0;
  conv_total_time = 0;
  fc_total_time = 0;
  pooling_total_time = 0;
  add_total_time = 0;
  mul_total_time = 0;
#endif

  TfLiteTensor* output = interpreter->output(0);

  // Process the inference results.
  int int8_t_un_dedo_score = output->data.uint8[kUnDedoIndex];
  int int8_t_rock_score = output->data.uint8[kRockIndex];
  int int8_t_tres_dedos_score = output->data.uint8[kTresDedosIndex];
  int int8_t_pulgar_score = output->data.uint8[kPulgarIndex];
  int int8_t_abierta_score = output->data.uint8[kAbiertaIndex];
  int int8_t_cerrado_score = output->data.uint8[kCerradoIndex];

  float un_dedo_score_f = (int8_t_un_dedo_score - output->params.zero_point) * output->params.scale;
  float rock_score_f = (int8_t_rock_score - output->params.zero_point) * output->params.scale;
  float tres_dedos_score_f = (int8_t_tres_dedos_score - output->params.zero_point) * output->params.scale;
  float pulgar_score_f = (int8_t_pulgar_score - output->params.zero_point) * output->params.scale;
  float abierta_score_f = (int8_t_abierta_score - output->params.zero_point) * output->params.scale;
  float cerrado_score_f = (int8_t_cerrado_score - output->params.zero_point) * output->params.scale;
  RespondToDetection(un_dedo_score_f, rock_score_f, tres_dedos_score_f, pulgar_score_f, abierta_score_f, cerrado_score_f);
}
