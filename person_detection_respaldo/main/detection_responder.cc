#include "detection_responder.h"
#include "tensorflow/lite/micro/micro_log.h"
//#include "uart_communication.h"

#include "esp_main.h"

char* RespondToDetection(float un_dedo_score, float rock_score, float tres_dedos_score, 
                        float pulgar_score, float abierta_score, float cerrado_score) {
  int un_dedo_score_int = (un_dedo_score) * 100 + 0.5;
  int rock_score_int = (rock_score) * 100 + 0.5;
  int tres_dedos_score_int = (tres_dedos_score) * 100 + 0.5;
  int pulgar_score_int = (pulgar_score) * 100 + 0.5;
  int abierta_score_int = (abierta_score) * 100 + 0.5;
  int cerrado_score_int = (cerrado_score) * 100 + 0.5;


 MicroPrintf("Scores - Un Dedo: %d%%, Rock: %d%%, Tres Dedos: %d%%, Pulgar: %d%%, Abierta: %d%%, Cerrado: %d%%",
              un_dedo_score_int, rock_score_int, tres_dedos_score_int, pulgar_score_int, abierta_score_int, cerrado_score_int);
  
  // Determinar el máximo score
  int max_score_int = un_dedo_score_int;
  char* data = "1";

  if (un_dedo_score_int > max_score_int) {data = "0"; max_score_int = un_dedo_score_int;}
  if (rock_score_int > max_score_int) {data = "1"; max_score_int = rock_score_int;}
  if (tres_dedos_score_int > max_score_int) {data = "2"; max_score_int = tres_dedos_score_int;}
  if (pulgar_score_int > max_score_int) {data = "3"; max_score_int = pulgar_score_int;}
  if (abierta_score_int > max_score_int) {data = "4"; max_score_int = abierta_score_int;}
  if (cerrado_score_int > max_score_int) {data = "5"; max_score_int = cerrado_score_int;}

  // uart_send_data(data); // Si necesitas enviar los datos a través de UART, descomenta esta línea.
  return data;
}