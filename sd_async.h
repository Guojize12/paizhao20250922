#pragma once
#include <Arduino.h>
#include "config.h"  

struct SdAsyncStats {
  uint32_t enq_ok = 0;
  uint32_t enq_drop = 0;
  uint32_t write_ok = 0;
  uint32_t write_fail = 0;
  uint32_t pool_free = 0;
  uint32_t pool_total = 0;
  uint32_t q_depth = 0;
  uint32_t q_max = 0;
  uint32_t task_stack_min = 0; // 最小剩余栈
  bool     running = false;
  bool     sd_ready = false;
};

bool sd_async_init();                      // 仅初始化数据结构（不启任务）
bool sd_async_start();                     // 启动后台写任务
void sd_async_stop(bool drain = true);     // 停止任务；drain=true会试着写完队列
bool sd_async_on_sd_ready();               // SD挂载完成后调用（或在 start 之前已挂载）
void sd_async_on_sd_lost();                // SD拔出/重挂前调用

// 提交一个写任务（内部会处理大于池块的缓冲：按块切分并按顺序追加写）
bool sd_async_submit(const char* path, const uint8_t* data, size_t len,
                     uint32_t timeout_ms = ASYNC_SD_SUBMIT_TIMEOUT_MS);

// 等待队列清空
bool sd_async_flush(uint32_t timeout_ms = ASYNC_SD_FLUSH_TIMEOUT_MS);

// 获取运行统计
void sd_async_get_stats(SdAsyncStats& out);

// 是否空闲（队列空且任务无在写）
bool sd_async_idle();