#include "scheduler.h"
#include "protocol.h"
#include "logging.h"
#if UPGRADE_ENABLE
#include "upgrade.h"
#endif

extern bool g_debugMode;

void protocol_scheduler(){
  uint32_t now=millis();
  static uint32_t lastAutoMs=0;

  if(now - g_lastHeartbeatSentMs >= g_normal_interval_sec*1000UL) platform_send_heartbeat();
  if(now - g_lastTimeReqSentMs >= g_normal_interval_sec*1000UL*10UL) platform_send_time_request();

  if(!g_sentPowerOn && now>3000) platform_send_power_on_status();
  if(!g_sentSimInfo && now>6000) platform_send_sim_info();

  if(!g_debugMode && g_work_interval_sec>0){
#if UPGRADE_ENABLE
    if(g_upg.state!=UPG_DOWNLOADING && g_upg.state!=UPG_FILE_INFO)
#endif
    {
      if(now - lastAutoMs >= g_work_interval_sec*1000UL){
        extern bool capture_and_process(uint8_t trigger);
        capture_and_process(TRIGGER_AUTO);
        lastAutoMs=now;
      }
    }
  }
  image_retry_scheduler();
#if UPGRADE_ENABLE
  upgrade_scheduler();
#endif
}