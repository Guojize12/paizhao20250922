#pragma once
#include <Arduino.h>
#include "config.h"

// Footer 布局（紧跟在固件内容末尾，不写入 OTA 分区）
// [MAGIC 'OTAF'(4)] [ver(1)=1] [algo(1)=1=ECDSA-P256] [digest_len(1)=32]
// [sig_len(2,BE)] [digest(32)] [sig(sig_len, DER)]
struct OtaFooterView {
  const uint8_t* base; // 指向 MAGIC 位置
  uint8_t ver;
  uint8_t algo;
  uint8_t digest_len; // 固定 32
  uint16_t sig_len;   // 一般 70~72
  const uint8_t* digest; // 32B
  const uint8_t* sig;    // DER
  uint32_t total_len() const { return 9u + 32u + sig_len; }
};

struct OtaSecureCtx {
  // 输入/进度
  uint32_t total_size = 0;   // 服务器宣告的总大小（含 Footer）
  uint32_t received   = 0;   // 已收（含 Footer）
  uint32_t content_written = 0; // 已写入 OTA 分区的“内容字节数”（不含 Footer）
  // SHA-256
  void* sha_ctx = nullptr;   // mbedtls_sha256_context*
  uint8_t sha_calc[32];      // 最终计算值
  // 尾缓冲（保存最后 <= OTA_FOOTER_MAX_SIZE 字节）
  uint8_t tail[OTA_FOOTER_MAX_SIZE];
  uint32_t tail_len = 0;

  // 断点位图
  uint32_t block_size = OTA_BITMAP_BLOCK_SIZE;
  uint32_t blocks_total = 0;
  uint32_t bitmap_bytes = 0;
  uint8_t* bitmap = nullptr;

  // Footer 解析结果（在 finalize 时得到）
  OtaFooterView footer{};

  // 一个短标签用于区分不同固件（来自版本号/文件名哈希）
  uint32_t tag_hash = 0;
};

// 初始化：total_size 为服务器宣告的总长度（含 Footer）；version_tag 用于断点位图命名（可传文件名/版本）
bool upgsec_begin(uint32_t total_size, const char* version_tag, uint32_t block_size);

// 每次收到一段数据就调用。file_offset 为本段在整个文件中的起始偏移；is_last 为最后一段。
// ota_write_cb 为你现有的 esp_ota_write 封装，返回写入字节数。
bool upgsec_update(uint32_t file_offset, const uint8_t* data, uint32_t len, bool is_last,
                   size_t (*ota_write_cb)(const uint8_t*, size_t), String& err);

// 完成后调用：用公钥验签。返回 true 表示验证通过。
bool upgsec_finalize_verify(const char* pubkey_pem, String& err);

// 断点位图 API（在分段成功写入后调用）
bool upgsec_mark_blocks(uint32_t file_offset, uint32_t len); // 根据覆盖范围置位
bool upgsec_save_bitmap();   // 持久化到 NVS
bool upgsec_load_bitmap(uint32_t total_size, const char* version_tag, uint32_t block_size);
bool upgsec_is_complete();   // 位图全部为 1

// 查询进度
void upgsec_get_progress(uint32_t& received_total, uint32_t& content_written);
void upgsec_reset(); // 释放资源（失败/重启时）