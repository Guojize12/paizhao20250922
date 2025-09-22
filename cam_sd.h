#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "config.h"

// 运行时配置（仅保留与SD写入相关）
struct RuntimeConfig{
  bool saveEnabled;
  bool asyncSDWrite;
};

extern RuntimeConfig g_cfg;
extern bool camera_ok;

// 闪光灯（保留）
void flashInit();
void flashSet(uint8_t d);
void flashOn();
void flashOff();

// 摄像头（保留）
bool init_camera_multi();
void deinit_camera_silent();
bool discard_frames(int n);
bool reinit_camera_with_params(framesize_t size,int quality);

// SD卡与异步写（保留）
void init_sd();
void periodic_sd_check();

// 拍照处理（只保存到SD）
bool capture_and_process(uint8_t trigger);

// 按键启动时等待释放（保留）
void wait_button_release_on_boot();