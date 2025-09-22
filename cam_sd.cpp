#include "cam_sd.h"
#include "logging.h"
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include "upgrade.h" // for prefs
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "config.h"
#include "sd_async.h"

// 外部符号
extern void platform_send_image(uint8_t trigger,const uint8_t* jpeg,uint32_t jpegLen,uint32_t frameIndex);

// SPI for SD
SPIClass sdSPI(VSPI);

// 运行状态
RunStats g_stats={0};
RuntimeConfig g_cfg={
  .sendBeforeSave=DEFAULT_SEND_BEFORE_SAVE,
  .saveEnabled=true,
  .sendEnabled=true,
  .frameHeader=false,
  .asyncSDWrite=false // 默认关闭异步写，可运行时设置 true
};
static uint32_t photo_index=1;
static uint8_t  g_flashDuty=DEFAULT_FLASH_DUTY;
static uint32_t last_params_saved_ms=0;

bool camera_ok=false;
static uint32_t camera_reinit_backoff_ms=0;
static uint32_t camera_next_reinit_allowed=0;
static uint32_t sd_remount_backoff_ms=3000;
static uint32_t sd_next_remount_allowed=0;
static const uint32_t CAMERA_BACKOFF_BASE=3000;
static const uint32_t CAMERA_BACKOFF_MAX=30000;
static const uint32_t SD_BACKOFF_MAX=30000;
uint32_t heap_min_reboot_dynamic=HEAP_MIN_REBOOT;

void flashInit(){
#if FLASH_MODE
  ledc_timer_config_t tcfg={
    .speed_mode=LEDC_LOW_SPEED_MODE,
    .duty_resolution=(ledc_timer_bit_t)8,
    .timer_num=FLASH_TIMER,
    .freq_hz=FLASH_FREQ_HZ,
    .clk_cfg=LEDC_AUTO_CLK
  };
  ledc_timer_config(&tcfg);
  ledc_channel_config_t ccfg={
    .gpio_num=FLASH_PIN,
    .speed_mode=LEDC_LOW_SPEED_MODE,
    .channel=FLASH_CHANNEL,
    .intr_type=LEDC_INTR_DISABLE,
    .timer_sel=FLASH_TIMER,
    .duty=0,
    .hpoint=0
#if ESP_IDF_VERSION_MAJOR>=5
    , .flags={ .output_invert=0 }
#endif
  };
  ledc_channel_config(&ccfg);
#else
  pinMode(FLASH_PIN,OUTPUT);
  digitalWrite(FLASH_PIN,LOW);
#endif
}
void flashSet(uint8_t d){
#if FLASH_MODE
  if(d>255)d=255;
  if(d>0 && d<5)d=5;
  g_flashDuty=d;
#else
  (void)d;
#endif
}
void flashOn(){
#if FLASH_MODE
  ledc_set_duty(LEDC_LOW_SPEED_MODE,FLASH_CHANNEL,g_flashDuty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE,FLASH_CHANNEL);
#else
  digitalWrite(FLASH_PIN,HIGH);
#endif
}
void flashOff(){
#if FLASH_MODE
  ledc_set_duty(LEDC_LOW_SPEED_MODE,FLASH_CHANNEL,0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE,FLASH_CHANNEL);
#else
  digitalWrite(FLASH_PIN,LOW);
#endif
}

camera_config_t make_config(framesize_t size,int xclk,int q){
  camera_config_t c;
  c.ledc_channel=LEDC_CHANNEL_0;
  c.ledc_timer=LEDC_TIMER_0;
  c.pin_d0=Y2_GPIO; c.pin_d1=Y3_GPIO; c.pin_d2=Y4_GPIO; c.pin_d3=Y5_GPIO;
  c.pin_d4=Y6_GPIO; c.pin_d5=Y7_GPIO; c.pin_d6=Y8_GPIO; c.pin_d7=Y9_GPIO;
  c.pin_xclk=XCLK_GPIO; c.pin_pclk=PCLK_GPIO; c.pin_vsync=VSYNC_GPIO;
  c.pin_href=HREF_GPIO; c.pin_sscb_sda=SIOD_GPIO; c.pin_sscb_scl=SIOC_GPIO;
  c.pin_pwdn=PWDN_GPIO; c.pin_reset=RESET_GPIO;
  c.xclk_freq_hz=xclk;
  c.pixel_format=PIXFORMAT_JPEG;
  if(psramFound()){
    c.frame_size=size; c.jpeg_quality=q; c.fb_count=1;
  }else{
    c.frame_size=(size>FRAMESIZE_VGA?FRAMESIZE_VGA:size);
    c.jpeg_quality=q+5; c.fb_count=1;
  }
  return c;
}

bool try_camera_init_once(framesize_t size,int xclk,int q){
  camera_config_t cfg=make_config(size,xclk,q);
  esp_err_t err=esp_camera_init(&cfg);
  if(err!=ESP_OK) LOG_WARN("[CAM] init err=0x%X",(unsigned)err);
  return err==ESP_OK;
}

bool init_camera_multi(){
  pinMode(PWDN_GPIO,OUTPUT); digitalWrite(PWDN_GPIO,LOW); delay(30);
  for(int i=0;i<INIT_RETRY_PER_CONFIG;i++){
    if(try_camera_init_once(FRAME_SIZE_PREF,20000000,JPEG_QUALITY_PREF)) return true;
    delay(120);
  }
  esp_camera_deinit(); delay(60);
  for(int i=0;i<INIT_RETRY_PER_CONFIG;i++){
    if(try_camera_init_once(FRAME_SIZE_FALLBACK,10000000,JPEG_QUALITY_FALLBACK)) return true;
    delay(150);
  }
  esp_camera_deinit();
  return false;
}

void deinit_camera_silent(){ esp_camera_deinit(); delay(50); }

bool discard_frames(int n){
  for(int i=0;i<n;i++){
    camera_fb_t *fb=esp_camera_fb_get();
    if(!fb) return false;
    esp_camera_fb_return(fb);
  }
  return true;
}

bool reinit_camera_with_params(framesize_t size,int quality){
  deinit_camera_silent();
  camera_config_t cfg=make_config(size,20000000,quality);
  if(esp_camera_init(&cfg)==ESP_OK){
    camera_ok=true; LOG_INFO("[CAM] reinit fast OK size=%d q=%d",(int)size,quality); return true;
  }
  deinit_camera_silent();
  cfg=make_config(size,10000000,quality+2);
  if(esp_camera_init(&cfg)==ESP_OK){
    camera_ok=true; LOG_INFO("[CAM] reinit slow OK size=%d q=%d",(int)size,quality+2); return true;
  }
  camera_ok=false; LOG_WARN("[CAM] reinit fail size=%d q=%d",(int)size,quality); return false;
}

void schedule_camera_backoff(){
  camera_reinit_backoff_ms = camera_reinit_backoff_ms? min<uint32_t>(camera_reinit_backoff_ms*2,CAMERA_BACKOFF_MAX):CAMERA_BACKOFF_BASE;
  camera_next_reinit_allowed=millis()+camera_reinit_backoff_ms;
}
void attempt_camera_reinit_with_backoff(){
  uint32_t now=millis();
  if(now<camera_next_reinit_allowed) return;
  deinit_camera_silent();
  camera_ok=init_camera_multi();
  if(!camera_ok) schedule_camera_backoff(); else camera_reinit_backoff_ms=0;
}

bool save_frame_to_sd_raw(const uint8_t* data,size_t len,uint32_t index){
  if(SD.cardType()==CARD_NONE) return false;
  uint64_t free=(SD.totalBytes()-SD.usedBytes());
  if(free/(1024*1024) < SD_MIN_FREE_MB) return false;
  char name[48]; snprintf(name,sizeof(name),"/photo_%05lu.jpg",(unsigned long)index);
  File f=SD.open(name,FILE_WRITE); if(!f) return false;
  size_t w=f.write(data,len); f.close(); return w==len;
}

// 统一入口：支持同步/异步写
bool save_frame_to_sd(camera_fb_t *fb,uint32_t index){
  if(!fb) return false;
  char name[48];
  snprintf(name,sizeof(name),"/photo_%05lu.jpg",(unsigned long)index);
  if(g_cfg.asyncSDWrite){
    // 异步方式，失败可选择兜底（同步/报警）
    if(sd_async_submit(name, fb->buf, fb->len)){
      return true;
    }else{
      LOG_WARN("[SDASYNC] enq fail, fallback to sync");
      // 兜底可选：同步写
      return save_frame_to_sd_raw(fb->buf, fb->len, index);
    }
  }else{
    // 同步写
    return save_frame_to_sd_raw(fb->buf, fb->len, index);
  }
}

void init_sd(){
  sdSPI.begin(SD_SCK,SD_MISO,SD_MOSI,SD_CS);
  if(!SD.begin(SD_CS,sdSPI)) LOG_WARN("[SD] init fail");
  else {
    // SD挂载成功后启动异步写系统（只需调用一次）
    static bool async_started = false;
    if(!async_started){
      sd_async_init();
      sd_async_start();
      async_started = true;
    }
    sd_async_on_sd_ready();
  }
}

void periodic_sd_check(){
  uint32_t now=millis();
  if(SD.cardType()!=CARD_NONE){ sd_remount_backoff_ms=3000; return; }
  if(now<sd_next_remount_allowed) return;
  sd_async_on_sd_lost(); // SD断开时通知异步系统
  init_sd();
  if(SD.cardType()==CARD_NONE){
    sd_remount_backoff_ms=min<uint32_t>(sd_remount_backoff_ms*2,SD_BACKOFF_MAX);
    sd_next_remount_allowed=now+sd_remount_backoff_ms;
  }else sd_remount_backoff_ms=3000;
}

void save_params_to_nvs(){
  prefs.putUChar("flashduty",g_flashDuty);
  prefs.putUInt("photo_idx",photo_index);
  last_params_saved_ms=millis();
}
void load_params_from_nvs(){
  g_flashDuty=prefs.getUChar("flashduty",DEFAULT_FLASH_DUTY);
  photo_index=prefs.getUInt("photo_idx",1);
  if(photo_index==0) photo_index=1;
  last_params_saved_ms=millis();
}

void handle_camera_failure(){
  g_stats.failed_captures++; g_stats.consecutive_capture_fail++;
  if(ENABLE_AUTO_REINIT && g_stats.consecutive_capture_fail>=RUNTIME_FAIL_REINIT_THRESHOLD)
    attempt_camera_reinit_with_backoff();
  if(g_stats.consecutive_capture_fail>=CAPTURE_FAIL_REBOOT_THRESHOLD){
    LOG_FATAL("[CAM] too many fails"); delay(500); ESP.restart();
  }
}
void handle_sd_failure(){
  g_stats.consecutive_sd_fail++;
  if(g_stats.consecutive_sd_fail>=SD_FAIL_REBOOT_THRESHOLD){
    LOG_FATAL("[SD] too many fails"); delay(500); ESP.restart();
  }
}
void check_and_reboot_on_low_heap(){
  uint32_t freeH=esp_get_free_heap_size();
  if(freeH<heap_min_reboot_dynamic){
    LOG_FATAL("[MEM] low"); delay(500); ESP.restart();
  }
}

uint8_t capture_once_internal(uint8_t trigger){
#if UPGRADE_ENABLE
  if(g_upg.state==UPG_DOWNLOADING || g_upg.state==UPG_FILE_INFO){
    if(g_debugMode) LOG_WARN("[CAP] blocked by upgrade"); return CR_CAMERA_NOT_READY;
  }
#endif
  if(!camera_ok) return CR_CAMERA_NOT_READY;
  if(DISCARD_FRAMES_EACH_SHOT>0) discard_frames(DISCARD_FRAMES_EACH_SHOT);
  flashOn();
#if FLASH_MODE
  delay(FLASH_WARM_MS);
#else
  delay(FLASH_ON_TIME_MS_DIGITAL);
#endif
  camera_fb_t *fb=esp_camera_fb_get();
  if(!fb){
    delay(20); fb=esp_camera_fb_get();
    if(!fb){ flashOff(); return CR_FRAME_GRAB_FAIL; }
  }
  uint32_t frame_len=fb->len;
  uint32_t index=photo_index;
  bool sdOk=true;

  if(g_cfg.sendEnabled && g_cfg.sendBeforeSave)
    platform_send_image(trigger,fb->buf,fb->len,index);
  if(g_cfg.saveEnabled) sdOk=save_frame_to_sd(fb,index);
  if(g_cfg.sendEnabled && !g_cfg.sendBeforeSave)
    platform_send_image(trigger,fb->buf,fb->len,index);

  if(sdOk || !g_cfg.saveEnabled){
    photo_index++;
    if(photo_index % SAVE_PARAMS_INTERVAL_IMAGES==0) save_params_to_nvs();
    else if(millis()-last_params_saved_ms >= NVS_MIN_SAVE_INTERVAL_MS) save_params_to_nvs();
  }
  esp_camera_fb_return(fb);
  flashOff();

  g_stats.total_captures++;
  g_stats.last_frame_size=frame_len;
  g_stats.last_capture_ms=millis();
  if(!sdOk && g_cfg.saveEnabled) return CR_SD_SAVE_FAIL;
  check_and_reboot_on_low_heap();
  return CR_OK;
}

bool capture_and_process(uint8_t trigger){
  uint8_t r=capture_once_internal(trigger);
  if(r!=CR_OK){
    if(r==CR_FRAME_GRAB_FAIL) handle_camera_failure();
    else if(r==CR_SD_SAVE_FAIL) handle_sd_failure();
    else if(r==CR_CAMERA_NOT_READY && camera_ok==false) attempt_camera_reinit_with_backoff();
    return false;
  }else{
    g_stats.consecutive_capture_fail=0;
    g_stats.consecutive_sd_fail=0;
  }
  return true;
}

void wait_button_release_on_boot(){
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  if(digitalRead(BUTTON_PIN)==LOW)
    while(digitalRead(BUTTON_PIN)==LOW) delay(10);
}