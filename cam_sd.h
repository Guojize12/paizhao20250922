#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "config.h"

struct RunStats{
  uint32_t total_captures;
  uint32_t failed_captures;
  uint32_t consecutive_capture_fail;
  uint32_t consecutive_sd_fail;
  uint32_t last_frame_size;
  uint32_t last_capture_ms;
  uint32_t last_sd_write_ms;
};
struct RuntimeConfig{
  bool sendBeforeSave;
  bool saveEnabled;
  bool sendEnabled;
  bool frameHeader;
  bool asyncSDWrite;
};

extern RunStats g_stats;
extern RuntimeConfig g_cfg;
extern uint32_t heap_min_reboot_dynamic;

extern bool camera_ok;

void flashInit();
void flashSet(uint8_t d);
void flashOn();
void flashOff();

bool init_camera_multi();
void deinit_camera_silent();
bool discard_frames(int n);
bool reinit_camera_with_params(framesize_t size,int quality);

void init_sd();
void periodic_sd_check();

void save_params_to_nvs();
void load_params_from_nvs();

void handle_camera_failure();
void handle_sd_failure();
void check_and_reboot_on_low_heap();

bool capture_and_process(uint8_t trigger);

void wait_button_release_on_boot();