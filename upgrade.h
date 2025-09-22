#pragma once
#include <Arduino.h>
#include <esp_ota_ops.h>
#include <mbedtls/md5.h>
#include <Preferences.h>   // 新增：声明 prefs 需要

#ifndef UPGRADE_ENABLE
#define UPGRADE_ENABLE 1
#endif

// 升级状态（按你现有定义保留原语义）
enum UpgradeState : uint8_t {
  UPG_NONE        = 0,
  UPG_REQ         = 1,
  UPG_FILE_INFO   = 2,
  UPG_DOWNLOADING = 3,
  UPG_READY_APPLY = 4,
  UPG_ERROR_MD5   = 5,
  UPG_ERROR_COMM  = 6
};

// 升级会话上下文（完全对齐 upgrade.cpp 中的实际字段用法）
struct UpgradeSession {
  UpgradeState state = UPG_NONE;

  uint32_t totalSize = 0;     // 服务器宣告的总大小（含Footer）
  uint32_t received  = 0;     // 已接收总字节（含Footer）

  // 兼容旧版MD5字段（现在用于保存服务端下发的MD5字符串，供位图tag等）
  char     md5_hex[33] = {0};
  uint8_t  md5_bin[16] = {0};
  bool     md5_bin_valid = false;

  // MD5上下文（旧版流程会用；现在 finalize 时不再使用，但保留字段兼容）
  mbedtls_md5_context md5_ctx;
  bool     md5_ctx_started = false;

  // OTA 写入会话
  bool otaBegun = false;
  bool finalizing = false;
  const esp_partition_t* updatePartition = nullptr;
  esp_ota_handle_t otaHandle = 0;

  // 拉取/重试与请求参数
  uint32_t requestBlockSize = 0; // 从 NVS 载入或默认 UPG_BLOCK_SIZE
  uint32_t currentOffset    = 0;
  uint8_t  retryInfoLeft    = 0;
  uint8_t  retryBlockLeft   = 0;
  bool     waitingInfo      = false;
  bool     waitingBlock     = false;
  uint32_t lastRequestMs    = 0;
};

// 全局会话（由 upgrade.cpp 定义）
extern UpgradeSession g_upg;

// 全局 NVS 首选项句柄（由 upgrade.cpp 定义）
extern Preferences prefs;

// 协议回调（由 protocol 调用）
void handle_file_info_response(const uint8_t* payload, uint16_t len);
void handle_file_block_response(const uint8_t* payload, uint16_t len);

// 外部触发与调度
void upgrade_set_req_and_reboot();
void upgrade_scheduler();

// 会话持久化/复位
void upgrade_save_state();
void upgrade_load_state();
void upgrade_reset_session();