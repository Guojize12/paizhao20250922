#include <Arduino.h>
#include "config.h"
#include "logging.h"
#include "protocol.h"
#include "cam_sd.h"
#include "scheduler.h"
#include "cli.h"
#if UPGRADE_ENABLE
#include "upgrade.h"
#include "esp_ota_ops.h"
#endif
#include <string.h>
#include "sd_async.h"

bool g_debugMode = true;
extern uint32_t g_lastHeartbeatSentMs;
extern uint32_t g_lastTimeReqSentMs;
extern bool g_sentPowerOn;
extern bool g_sentSimInfo;
extern char g_sim_iccid[21];

void print_boot_status(bool cam_ok){
  if(!g_debugMode) return;
  Serial.println("===== Boot =====");
  Serial.printf("Build: %s %s\n",__DATE__,__TIME__);
  Serial.printf("Cam: %s\n",cam_ok?"OK":"FAIL");
  Serial.printf("Heap: %u\n",(unsigned)esp_get_free_heap_size());
#if UPGRADE_ENABLE
  Serial.printf("Upgrade state=%u\n",(unsigned)g_upg.state);
#endif
  Serial.println("================");
}

void setup(){
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(50);
  delay(300);
  if(g_debugMode) Serial.println("\n=== ESP32-CAM Protocol (Upgrade + GetImage) ===");
  wait_button_release_on_boot();
  pinMode(BUTTON_PIN,INPUT_PULLUP);

#if UPGRADE_ENABLE
  prefs.begin("esp32cam",false);
#else
  // 若关闭升级，请在此自行实例化并管理 Preferences
#endif
  load_params_from_nvs();
#if UPGRADE_ENABLE
  upgrade_load_state();
#endif
  extern uint32_t heap_min_reboot_dynamic;
  if(psramFound()) heap_min_reboot_dynamic = HEAP_MIN_REBOOT/2;
  memset(g_sim_iccid,'0',20); g_sim_iccid[20]='\0';
  init_sd();
  camera_ok=init_camera_multi();
  flashInit(); flashOff();
  if(camera_ok && DISCARD_FRAMES_ON_START>0) discard_frames(DISCARD_FRAMES_ON_START);

#if UPGRADE_ENABLE
  const esp_partition_t* nextp=esp_ota_get_next_update_partition(nullptr);
  if(nextp){
    if(nextp->size>4096) UPG_MAX_TOTAL_SIZE = nextp->size - 4096;
    if(g_debugMode) LOG_INFO("[UPG] next part size=%lu, max_total=%lu",
      (unsigned long)nextp->size,(unsigned long)UPG_MAX_TOTAL_SIZE);
#if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    esp_ota_mark_app_valid_cancel_rollback();
#endif
  }else{
    if(g_debugMode) LOG_WARN("[UPG] no OTA partition; disable upgrade");
    g_upg.state=UPG_ERROR_COMM; upgrade_save_state();
  }
  if(g_upg.state==UPG_REQ){
    g_upg.state=UPG_FILE_INFO; upgrade_save_state();
  }
#endif

  print_boot_status(camera_ok);
  extern void print_help(); extern void print_modes(); extern void print_version();
  print_help(); print_modes(); print_version();
  platform_send_heartbeat();
}

void loop(){
  platform_serial_poll();
  handle_serial_ascii_commands();
  static int lastBtn=HIGH;
  int cur=digitalRead(BUTTON_PIN);
  if(cur==LOW && lastBtn==HIGH) capture_and_process(TRIGGER_BUTTON);
  lastBtn=cur;
  periodic_sd_check();
  protocol_scheduler();
  delay(2);
}