#pragma once
#include <Arduino.h>
#include "config.h"

extern uint32_t g_normal_interval_sec;
extern uint32_t g_work_interval_sec;
extern PlatformAddress g_server_addr;

extern bool camera_ok;

struct PendingImage{
  uint8_t *payload;
  uint32_t payloadLen;
  uint32_t imageLen;
  uint8_t  pid;
  uint8_t  trigger;
  uint8_t  retries_left;
  uint32_t last_send_ms;
  uint32_t frame_index;
};
extern PendingImage* g_pending_image;

extern uint8_t g_pid_counter;

// 供调度器使用的时序标志（在 protocol.cpp 定义）
extern uint32_t g_lastHeartbeatSentMs;
extern uint32_t g_lastTimeReqSentMs;
extern bool g_sentPowerOn;
extern bool g_sentSimInfo;

void platform_send_packet(char op,uint16_t cmd,uint8_t pid,const uint8_t* payload,uint32_t payloadLen);
void platform_send_ack_W(uint16_t cmd,uint8_t pid,uint8_t status);
void platform_send_ack_R(uint16_t cmd,uint8_t pid,uint8_t status);

void platform_send_heartbeat();
void platform_send_time_request();
void platform_send_power_on_status();
void platform_send_sim_info();

void platform_apply_time(const uint8_t* d,uint16_t len);
uint8_t platform_set_intervals(const uint8_t* d,uint16_t len);
uint8_t platform_set_address(const uint8_t* d,uint16_t len);
void platform_get_address(uint8_t pid,const uint8_t* d,uint16_t len);

void platform_send_image(uint8_t trigger,const uint8_t* jpeg,uint32_t jpegLen,uint32_t frameIndex);
void image_retry_scheduler();
void image_handle_ack(uint8_t pid,const uint8_t* payload,uint16_t len);

void platform_handle_get_image_request(const uint8_t* payload, uint16_t len, uint8_t pid);

size_t try_parse_one_platform_packet(uint8_t* buf,size_t len);
void platform_serial_poll();

void print_protocol_status();