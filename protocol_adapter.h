#pragma once
#include <Arduino.h>
#include "config.h"

// 若未在 config.h 定义，给出安全默认值
#ifndef PROTO_MIN_SEND_INTERVAL_MS
#define PROTO_MIN_SEND_INTERVAL_MS 150
#endif

// 这两个是你“原来的实现逻辑”，已在 protocol.cpp 中改名为 *_impl
void platform_send_packet_impl(char op, uint16_t cmd, uint8_t pid,
                               const uint8_t* payload, uint32_t payloadLen);
void platform_serial_poll_impl();

// 适配层对外导出（其它模块仍然用这两个同名函数）
void platform_send_packet(char op, uint16_t cmd, uint8_t pid,
                          const uint8_t* payload, uint32_t payloadLen);
void platform_serial_poll();

// 解析统计钩子（在 protocol.cpp 的解析分支中调用）
extern "C" void proto_adapter_on_crc_err();
extern "C" void proto_adapter_on_discard();
extern "C" void proto_adapter_on_rx_ok();

// 计数导出（可用于调试/状态查询）
extern "C" uint32_t proto_get_rx_crc_err();
extern "C" uint32_t proto_get_rx_discard();
extern "C" uint32_t proto_get_rx_frames();
extern "C" uint32_t proto_get_tx_frames();
extern "C" uint8_t  proto_get_busy();