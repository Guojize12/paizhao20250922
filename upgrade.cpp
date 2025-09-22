#include "upgrade.h"
#if UPGRADE_ENABLE
#include "logging.h"
#include "util.h"
#include "config.h"
#include "crc16.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include <string.h>

#include "upgrade_secure.h"  // 新增：安全 OTA（尾部 Footer + SHA256 + 签名）
#include "ota_pubkey.h"      // 新增：公钥（请替换为你的实际 PEM）

UpgradeSession g_upg;
Preferences prefs;

// 由 protocol 提供
extern void platform_send_packet(char op,uint16_t cmd,uint8_t pid,const uint8_t* payload,uint32_t payloadLen);
extern uint8_t g_pid_counter;

// 仅用于兼容：解析 MD5 HEX（我们不再计算MD5，但用其字符串做位图 version_tag）
static bool parse_md5_hex(const char* hex,uint8_t out16[16]){
  if(!hex) return false;
  for(int i=0;i<32;i++){ char c=hex[i]; if(!c||!isxdigit((unsigned char)c)) return false; }
  auto cv=[](char x)->uint8_t{
    if(x>='0'&&x<='9') return x-'0';
    if(x>='a'&&x<='f') return x-'a'+10;
    if(x>='A'&&x<='F') return x-'A'+10;
    return 0;
  };
  for(int i=0;i<16;i++) out16[i]=(cv(hex[i*2])<<4)|cv(hex[i*2+1]);
  return true;
}

void upgrade_save_state(){
  prefs.putUChar(NVS_KEY_UP_STATE,(uint8_t)g_upg.state);
  prefs.putUInt(NVS_KEY_UP_TOTAL,g_upg.totalSize);
  prefs.putUInt(NVS_KEY_UP_RECV,g_upg.received);
  prefs.putString(NVS_KEY_UP_MD5HEX,String(g_upg.md5_hex));
  prefs.putUInt(NVS_KEY_UP_BLKSZ,g_upg.requestBlockSize);
}

void upgrade_reset_session(){
  // 不再使用 MD5 计算，保持清理逻辑不变（兼容旧字段）
  g_upg.state=UPG_NONE;
  g_upg.totalSize=0;
  g_upg.received=0;
  g_upg.md5_hex[0]='\0';
  g_upg.md5_bin_valid=false;
  g_upg.otaBegun=false;
  g_upg.finalizing=false;
  upgsec_reset(); // 新增：清理安全OTA上下文（位图/尾缓冲等）
  upgrade_save_state();
}

void upgrade_load_state(){
  g_upg.state = (UpgradeState)prefs.getUChar(NVS_KEY_UP_STATE,UPG_NONE);
  g_upg.totalSize = prefs.getUInt(NVS_KEY_UP_TOTAL,0);
  g_upg.received  = prefs.getUInt(NVS_KEY_UP_RECV,0);
  String md5s = prefs.getString(NVS_KEY_UP_MD5HEX,"");
  memset(g_upg.md5_hex,0,sizeof(g_upg.md5_hex));
  if(md5s.length()==32) md5s.toCharArray(g_upg.md5_hex,33);
  g_upg.requestBlockSize = prefs.getUInt(NVS_KEY_UP_BLKSZ,UPG_BLOCK_SIZE);
  if(g_upg.requestBlockSize==0 || g_upg.requestBlockSize>8192) g_upg.requestBlockSize=UPG_BLOCK_SIZE;
  g_upg.received = (g_upg.received>g_upg.totalSize)?0:g_upg.received;
  g_upg.currentOffset=g_upg.received;
  g_upg.retryInfoLeft=UPG_INFO_MAX_RETRY;
  g_upg.retryBlockLeft=UPG_BLOCK_MAX_RETRY;
  g_upg.waitingInfo=false; g_upg.waitingBlock=false;
  g_upg.md5_bin_valid=false; // 兼容字段
  g_upg.otaBegun=false; g_upg.finalizing=false;
}

void upgrade_set_req_and_reboot(){
  g_upg.state=UPG_REQ; upgrade_save_state();
  if(g_debugMode) LOG_INFO("[UPG] rebooting in %ums",UPG_REBOOT_DELAY_MS);
  delay(UPG_REBOOT_DELAY_MS);
  ESP.restart();
}

void send_file_info_request(){
  platform_send_packet('R',CMD_FILE_INFO_REQ,g_pid_counter++,nullptr,0);
  g_upg.waitingInfo=true; g_upg.lastRequestMs=millis();
  if(g_debugMode) LOG_INFO("[UPG] send file info");
}

// 写入回调：供安全OTA在 upgsec_update 中调用，屏蔽 Footer
static size_t upg_ota_write_cb(const uint8_t* p, size_t n){
  if(!g_upg.otaBegun) return 0;
  return (esp_ota_write(g_upg.otaHandle, p, n) == ESP_OK) ? n : 0;
}

void handle_file_info_response(const uint8_t* payload,uint16_t len){
  g_upg.waitingInfo=false;
  if(len<37){
    if(g_debugMode) LOG_WARN("[UPG] info resp short");
    if(g_upg.retryInfoLeft>0){ g_upg.retryInfoLeft--; send_file_info_request(); }
    else { g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); }
    return;
  }
  uint8_t flag=payload[0];
  uint32_t total=get_u32_be(payload+1);
  if(flag==0){
    if(g_debugMode) LOG_INFO("[UPG] no file");
    upgrade_reset_session(); return;
  }
  if(total==0 || total>UPG_MAX_TOTAL_SIZE){
    if(g_debugMode) LOG_WARN("[UPG] size invalid %lu > %lu",(unsigned long)total,(unsigned long)UPG_MAX_TOTAL_SIZE);
    g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); return;
  }
  char md5hex[33]; memset(md5hex,0,sizeof(md5hex));
  memcpy(md5hex,payload+5,32);

  // 如果之前已有残留进度，这里清零后按位图续传（由 upgsec_load_bitmap 决定）
  if(g_upg.received>0){
    if(g_debugMode) LOG_INFO("[UPG] discard previous partial, restart");
    g_upg.received=0;
  }

  strncpy(g_upg.md5_hex,md5hex,32); g_upg.md5_hex[32]='\0';
  parse_md5_hex(g_upg.md5_hex,g_upg.md5_bin); g_upg.md5_bin_valid=true; // 仅保留旧字段，不参与验收
  g_upg.totalSize=total; g_upg.currentOffset=g_upg.received;
  g_upg.state=UPG_DOWNLOADING; g_upg.retryBlockLeft=UPG_BLOCK_MAX_RETRY;

  // 打开 OTA 会话：使用 OTA_SIZE_UNKNOWN，避免“Footer 导致 total 超分区”误判
  g_upg.updatePartition = esp_ota_get_next_update_partition(nullptr);
  if(!g_upg.updatePartition){
    LOG_WARN("[UPG] no update partition");
    g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); return;
  }
  if(esp_ota_begin(g_upg.updatePartition, OTA_SIZE_UNKNOWN, &g_upg.otaHandle)!=ESP_OK){
    LOG_WARN("[UPG] ota begin fail");
    g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); return;
  }
  g_upg.otaBegun=true;

  // 初始化/加载断点位图（用 md5_hex 作为版本标签，不改协议）
  upgsec_load_bitmap(total, g_upg.md5_hex[0]? g_upg.md5_hex : "fw", UPG_BLOCK_SIZE);

  if(g_debugMode) LOG_INFO("[UPG] info OK total=%lu md5=%s",(unsigned long)g_upg.totalSize,g_upg.md5_hex);
  upgrade_save_state();
}

void send_file_block_request(){
  if(g_upg.currentOffset>=g_upg.totalSize){
    g_upg.state=UPG_READY_APPLY; upgrade_save_state(); return;
  }
  uint32_t want=g_upg.requestBlockSize;
  if(g_upg.currentOffset+want>g_upg.totalSize) want=g_upg.totalSize-g_upg.currentOffset;
  uint8_t req[8];
  put_u32_be(req+0,g_upg.currentOffset);
  put_u32_be(req+4,want);
  platform_send_packet('R',CMD_FILE_BLOCK_REQ,g_pid_counter++,req,8);
  g_upg.waitingBlock=true; g_upg.lastRequestMs=millis();
  if(g_debugMode) LOG_INFO("[UPG] req block off=%lu len=%lu",(unsigned long)g_upg.currentOffset,(unsigned long)want);
}

void handle_file_block_response(const uint8_t* payload,uint16_t len){
  g_upg.waitingBlock=false;
  if(len<1){ if(g_upg.retryBlockLeft>0){ g_upg.retryBlockLeft--; send_file_block_request(); } else { g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); } return; }
  uint8_t st=payload[0];
  if(st==0){ if(g_upg.retryBlockLeft>0){ g_upg.retryBlockLeft--; send_file_block_request(); } else { g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); } return; }
  if(len<1+4+4){ if(g_upg.retryBlockLeft>0){ g_upg.retryBlockLeft--; send_file_block_request(); } else { g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); } return; }
  uint32_t start=get_u32_be(payload+1);
  uint32_t dlen=get_u32_be(payload+5);
  if(len!=1+4+4+dlen || start!=g_upg.currentOffset){
    if(g_upg.retryBlockLeft>0){ g_upg.retryBlockLeft--; send_file_block_request(); }
    else { g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); }
    return;
  }
  const uint8_t* data=payload+9;

  if(!g_upg.otaBegun){
    g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); return;
  }

  // 使用安全 OTA：写入时自动保留尾部，避免把 Footer 写入 OTA 分区；同时做 SHA-256
  bool is_last = (start + dlen >= g_upg.totalSize);
  String err;
  if(!upgsec_update(start, data, dlen, is_last, upg_ota_write_cb, err)){
    // 出错：终止 OTA 会话
    esp_ota_abort(g_upg.otaHandle);
    g_upg.otaBegun=false;
    upgsec_reset();
    g_upg.state=UPG_ERROR_COMM; upgrade_save_state();
    if(g_debugMode) LOG_WARN("[UPG] update fail: %s", err.c_str());
    return;
  }

  // 进度与重试
  g_upg.currentOffset+=dlen; g_upg.received=g_upg.currentOffset;
  g_upg.retryBlockLeft=UPG_BLOCK_MAX_RETRY;
  upgrade_save_state();

  // 保存位图（可降频；这里简单每块保存一次）
  upgsec_save_bitmap();

  if(g_debugMode){
    uint32_t pct=(uint64_t)g_upg.received*100/g_upg.totalSize;
    LOG_INFO("[UPG] %lu/%lu (%u%%)",(unsigned long)g_upg.received,(unsigned long)g_upg.totalSize,pct);
  }

  if(g_upg.currentOffset>=g_upg.totalSize){
    // 所有字节接收完成，但还未验签 → UPG_READY_APPLY 交给 finalize 验证 + end
    g_upg.state=UPG_READY_APPLY; upgrade_save_state();
  }else{
    send_file_block_request();
  }
}

void upgrade_finalize_and_apply(){
  if(!g_upg.otaBegun){
    g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); return;
  }

  // 验签：比对 SHA-256 摘要并做 ECDSA-P256 签名验证
  String verr;
  if(!upgsec_finalize_verify(OTA_PUBKEY_PEM, verr)){
    // 签名/摘要失败 → 映射为 UPG_ERROR_MD5（兼容你现有语义）
    esp_ota_abort(g_upg.otaHandle);
    g_upg.otaBegun=false;
    upgsec_reset();
    g_upg.state=UPG_ERROR_MD5; upgrade_save_state();
    if(g_debugMode) LOG_WARN("[UPG] verify fail: %s", verr.c_str());
    return;
  }

  // 结束 OTA 会话并设置为下一次启动分区
  if(esp_ota_end(g_upg.otaHandle)!=ESP_OK){
    upgsec_reset();
    g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); return;
  }
  if(esp_ota_set_boot_partition(g_upg.updatePartition)!=ESP_OK){
    upgsec_reset();
    g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); return;
  }
#if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
  esp_ota_mark_app_valid_cancel_rollback();
#endif
  upgsec_reset();
  if(g_debugMode) LOG_INFO("[UPG] success reboot");
  upgrade_reset_session();
  delay(300);
  ESP.restart();
}

void upgrade_scheduler(){
  switch(g_upg.state){
    case UPG_NONE: break;
    case UPG_REQ:
      if(!esp_ota_get_next_update_partition(nullptr)){
        g_upg.state=UPG_ERROR_COMM; upgrade_save_state();
        if(g_debugMode) LOG_WARN("[UPG] no OTA partition");
        break;
      }
      g_upg.state=UPG_FILE_INFO; upgrade_save_state();
      send_file_info_request();
      break;
    case UPG_FILE_INFO:
      if(!g_upg.waitingInfo) send_file_info_request();
      else if(millis()-g_upg.lastRequestMs>UPG_INFO_RESP_TIMEOUT){
        if(g_upg.retryInfoLeft>0){ g_upg.retryInfoLeft--; send_file_info_request(); }
        else { g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); }
      }
      break;
    case UPG_DOWNLOADING:
      if(!g_upg.waitingBlock) send_file_block_request();
      else if(millis()-g_upg.lastRequestMs>UPG_BLOCK_RESP_TIMEOUT){
        if(g_upg.retryBlockLeft>0){ g_upg.retryBlockLeft--; send_file_block_request(); }
        else { g_upg.state=UPG_ERROR_COMM; upgrade_save_state(); }
      }
      break;
    case UPG_READY_APPLY:
      upgrade_finalize_and_apply();
      break;
    case UPG_ERROR_MD5:
    case UPG_ERROR_COMM:
      break;
  }
}
#endif