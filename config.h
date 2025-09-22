#pragma once
#include <Arduino.h>
#include "esp_camera.h"

// ===== 异步SD写与内存池（务必放在最前，确保其它头文件引用这些宏时已定义）=====
#ifndef ASYNC_SD_ENABLE
#define ASYNC_SD_ENABLE 1
#endif

#ifndef ASYNC_SD_POOL_BLOCK_SIZE
#define ASYNC_SD_POOL_BLOCK_SIZE (256 * 1024)
#endif

#ifndef ASYNC_SD_POOL_BLOCKS
#define ASYNC_SD_POOL_BLOCKS 3
#endif

#ifndef ASYNC_SD_QUEUE_LENGTH
#define ASYNC_SD_QUEUE_LENGTH 8
#endif

#ifndef ASYNC_SD_TASK_STACK
#define ASYNC_SD_TASK_STACK 4096
#endif

#ifndef ASYNC_SD_TASK_PRIO
#define ASYNC_SD_TASK_PRIO 3
#endif

#ifndef ASYNC_SD_MAX_PATH
#define ASYNC_SD_MAX_PATH 96
#endif

#ifndef ASYNC_SD_SUBMIT_TIMEOUT_MS
#define ASYNC_SD_SUBMIT_TIMEOUT_MS 50
#endif

#ifndef ASYNC_SD_FLUSH_TIMEOUT_MS
#define ASYNC_SD_FLUSH_TIMEOUT_MS 5000
#endif
// ===== 异步SD写与内存池 END =====

// === 开关 ===
#define UPGRADE_ENABLE 1

// === 基本参数 ===
#define SERIAL_BAUD                    115200
#define DEFAULT_FLASH_DUTY             80
#define FRAME_SIZE_PREF                FRAMESIZE_SVGA
#define FRAME_SIZE_FALLBACK            FRAMESIZE_VGA
#define JPEG_QUALITY_PREF              15
#define JPEG_QUALITY_FALLBACK          20
#define INIT_RETRY_PER_CONFIG          3
#define RUNTIME_FAIL_REINIT_THRESHOLD  3
#define CAPTURE_FAIL_REBOOT_THRESHOLD  10
#define SD_FAIL_REBOOT_THRESHOLD       10
#define HEAP_MIN_REBOOT                10000
#define SD_MIN_FREE_MB                 5
#define DISCARD_FRAMES_ON_START        3
#define DISCARD_FRAMES_EACH_SHOT       0
#define ENABLE_AUTO_REINIT             1
#define ENABLE_STATS_LOG               1
#define ENABLE_FRAME_HEADER            0
#define ENABLE_ASYNC_SD_WRITE          0
#define SAVE_PARAMS_INTERVAL_IMAGES    50
#define DEFAULT_SEND_BEFORE_SAVE       1
#define FLASH_MODE                     1
#define FLASH_WARM_MS                  60
#define FLASH_ON_TIME_MS_DIGITAL       40
#define HEAP_WARN_THRESHOLD            16000
#define HEAP_LARGEST_BLOCK_WARN        12000
static constexpr uint32_t NVS_MIN_SAVE_INTERVAL_MS = 60UL * 1000UL;

#define CR_OK                0
#define CR_CAMERA_NOT_READY  1
#define CR_FRAME_GRAB_FAIL   2
#define CR_SD_SAVE_FAIL      3

#ifndef PROTO_MIN_SEND_INTERVAL_MS
#define PROTO_MIN_SEND_INTERVAL_MS 150  // 最小发送间隔(ms)，设置为0可完全关闭节流
#endif

// ===== OTA 安全与断点位图（不改协议、仅改本地升级实现与固件包格式）=====
#ifndef OTA_SECURE_ENABLE
#define OTA_SECURE_ENABLE 1
#endif

#ifndef OTA_BITMAP_BLOCK_SIZE
#define OTA_BITMAP_BLOCK_SIZE 4096   // 位图块大小（与平台打包一致）
#endif

#ifndef OTA_FOOTER_MAX_SIZE
#define OTA_FOOTER_MAX_SIZE 384      // 足够容纳 P-256 ECDSA DER 签名的 Footer
#endif

// === 引脚 ===
#define PWDN_GPIO 32
#define RESET_GPIO -1
#define XCLK_GPIO 0
#define SIOD_GPIO 26
#define SIOC_GPIO 27
#define Y9_GPIO 35
#define Y8_GPIO 34
#define Y7_GPIO 39
#define Y6_GPIO 36
#define Y5_GPIO 21
#define Y4_GPIO 19
#define Y3_GPIO 18
#define Y2_GPIO 5
#define VSYNC_GPIO 25
#define HREF_GPIO 23
#define PCLK_GPIO 22

// SD Pins
#define SD_CS   13
#define SD_SCK  14
#define SD_MISO 2
#define SD_MOSI 15

// Flash
#define FLASH_PIN 4
#if FLASH_MODE
  #define FLASH_FREQ_HZ      5000
  #define FLASH_RES_BITS     8
  #define FLASH_TIMER        LEDC_TIMER_1
  #define FLASH_SPEED_MODE   LEDC_LOW_SPEED_MODE
  #define FLASH_CHANNEL      LEDC_CHANNEL_4
#endif

// Button
#define BUTTON_PIN 12

// === 平台协议版本/型号 ===
#define PLATFORM_VER        0x5B
#define PLATFORM_DMODEL     0x1F

// 上行
#define CMD_HEARTBEAT_REQ       0x0000
#define CMD_TIME_SYNC_REQ       0x0001
#define CMD_POWER_ON_STATUS     0x0002
#define CMD_FILE_INFO_REQ       0x0003
#define CMD_FILE_BLOCK_REQ      0x0004
#define CMD_SIM_INFO            0x0007
// 下行
#define CMD_S_HEARTBEAT         0x8000
#define CMD_S_TIME_SYNC         0x8001
#define CMD_S_SET_INTERVAL      0x8002
#define CMD_S_SET_ADDR          0x8003
#define CMD_S_GET_ADDR          0x8013
#define CMD_S_PARAM_RWX         0x8014
#define CMD_REMOTE_UPGRADE_CMD  0x8005
#define CMD_REMOTE_FUNCTION_CMD 0x8006
#define CMD_S_GET_IMAGE         0x8015
// 图片上传
#define CMD_IMAGE_UPLOAD        0x1F00

// 回复码
#define RESP_OK          0x00
#define RESP_FAIL        0x01
#define RESP_UNSUPPORT   0x02
#define RESP_PARSE_FAIL  0xFF

// 间隔范围
static const uint32_t NORMAL_INTERVAL_MIN_SEC = 1;
static const uint32_t NORMAL_INTERVAL_MAX_SEC = 24UL*3600UL;
static const uint32_t WORK_INTERVAL_MIN_SEC   = 1;
static const uint32_t WORK_INTERVAL_MAX_SEC   = 24UL*3600UL;

// 触发类型
#define TRIGGER_AUTO   0
#define TRIGGER_BUTTON 1
#define TRIGGER_REMOTE 2

// 主动图片上传
#define IMAGE_MAX_LEN            65000
#define IMAGE_META_LEN           16
#define IMAGE_ACK_TIMEOUT_MS     5000
#define IMAGE_MAX_RETRIES        2
#define IMAGE_MIN_INTERVAL_MS    10000
#define PLATFORM_SLAVE_ID        0x00000001
#define GET_IMAGE_MAX_LEN        65000

// 升级常量
#if UPGRADE_ENABLE
static const uint32_t UPG_BLOCK_RESP_TIMEOUT  = 5000;
static const uint8_t  UPG_BLOCK_MAX_RETRY     = 3;
static const uint8_t  UPG_INFO_MAX_RETRY      = 3;
static const uint32_t UPG_REBOOT_DELAY_MS     = 500;
static uint32_t UPG_MAX_TOTAL_SIZE            = 8*1024*1024; // 动态修正
static const uint32_t UPG_INFO_RESP_TIMEOUT   = 5000;
static const uint32_t UPG_BLOCK_SIZE          = 1024;

static const char* NVS_KEY_UP_STATE  = "up_state";
static const char* NVS_KEY_UP_TOTAL  = "up_total";
static const char* NVS_KEY_UP_RECV   = "up_recv";
static const char* NVS_KEY_UP_MD5HEX = "up_md5";
static const char* NVS_KEY_UP_BLKSZ  = "up_blksz";
#endif

// 地址结构
struct PlatformAddress{
  bool valid;
  char host[128];
  uint32_t port;
};