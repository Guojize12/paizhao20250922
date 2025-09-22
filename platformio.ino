#include <Arduino.h>
#include "config.h"
#include "cam_sd.h"

void setup(){
  Serial.begin(SERIAL_BAUD);
  delay(200);
  wait_button_release_on_boot();
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  init_sd();
  camera_ok = init_camera_multi();
  flashInit(); flashOff();

  if(camera_ok && DISCARD_FRAMES_ON_START>0){
    discard_frames(DISCARD_FRAMES_ON_START);
  }

  Serial.printf("Boot: cam=%s, heap=%u\n", camera_ok?"OK":"FAIL", (unsigned)esp_get_free_heap_size());
}

void loop(){
  static int lastBtn = HIGH;
  int cur = digitalRead(BUTTON_PIN);
  if(cur==LOW && lastBtn==HIGH){
    capture_and_process(TRIGGER_BUTTON);
  }
  lastBtn = cur;

  periodic_sd_check();
  delay(2);
}