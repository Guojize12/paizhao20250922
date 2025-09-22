#include "cam_sd.h"
#include <SPI.h>
#include <SD.h>
#include "config.h"
#include "sd_async.h"

// SPI for SD
SPIClass sdSPI(VSPI);

// 运行时配置：仅保留与SD写入相关
RuntimeConfig g_cfg = {
  .saveEnabled   = true,
  .asyncSDWrite  = true  // 默认启用异步写
};

static uint32_t photo_index = 1;
static uint8_t  g_flashDuty = DEFAULT_FLASH_DUTY;

bool camera_ok = false;
static uint32_t camera_reinit_backoff_ms = 0;
static uint32_t camera_next_reinit_allowed = 0;
static uint32_t sd_remount_backoff_ms = 3000;
static uint32_t sd_next_remount_allowed = 0;
static const uint32_t CAMERA_BACKOFF_BASE = 3000;
static const uint32_t CAMERA_BACKOFF_MAX  = 30000;
static const uint32_t SD_BACKOFF_MAX      = 30000;

// 闪光灯
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

// 摄像头配置/初始化
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

// SD 保存
static bool save_frame_to_sd_raw(const uint8_t* data,size_t len,uint32_t index){
  if(SD.cardType()==CARD_NONE) return false;
  uint64_t free=(SD.totalBytes()-SD.usedBytes());
  if(free/(1024*1024) < SD_MIN_FREE_MB) return false;
  char name[48]; snprintf(name,sizeof(name),"/photo_%05lu.jpg",(unsigned long)index);
  File f=SD.open(name,FILE_WRITE); if(!f) return false;
  size_t w=f.write(data,len); f.close(); return w==len;
}

// 同步/异步统一入口
static bool save_frame_to_sd(camera_fb_t *fb,uint32_t index){
  if(!fb) return false;
  char name[48];
  snprintf(name,sizeof(name),"/photo_%05lu.jpg",(unsigned long)index);
  if(g_cfg.asyncSDWrite){
    if(sd_async_submit(name, fb->buf, fb->len)){
      return true;
    }else{
      LOG_WARN("[SDASYNC] enq fail, fallback to sync");
      return save_frame_to_sd_raw(fb->buf, fb->len, index);
    }
  }else{
    return save_frame_to_sd_raw(fb->buf, fb->len, index);
  }
}

// SD 初始化与周期检查
void init_sd(){
  sdSPI.begin(SD_SCK,SD_MISO,SD_MOSI,SD_CS);
  if(!SD.begin(SD_CS,sdSPI)) {
    LOG_WARN("[SD] init fail");
  } else {
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
  sd_async_on_sd_lost();
  init_sd();
  if(SD.cardType()==CARD_NONE){
    sd_remount_backoff_ms=min<uint32_t>(sd_remount_backoff_ms*2,SD_BACKOFF_MAX);
    sd_next_remount_allowed=now+sd_remount_backoff_ms;
  }else sd_remount_backoff_ms=3000;
}

// 单次拍照
static uint8_t capture_once_internal(uint8_t /*trigger*/){
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

  if(g_cfg.saveEnabled) sdOk=save_frame_to_sd(fb,index);

  if(sdOk || !g_cfg.saveEnabled){
    photo_index++;
  }

  esp_camera_fb_return(fb);
  flashOff();

  (void)frame_len; // 若需要可用于日志
  if(!sdOk && g_cfg.saveEnabled) return CR_SD_SAVE_FAIL;
  return CR_OK;
}

bool capture_and_process(uint8_t trigger){
  uint8_t r=capture_once_internal(trigger);
  if(r!=CR_OK){
    if(r==CR_CAMERA_NOT_READY && camera_ok==false){
      attempt_camera_reinit_with_backoff();
    }else if(r==CR_FRAME_GRAB_FAIL && ENABLE_AUTO_REINIT){
      attempt_camera_reinit_with_backoff();
    }
    return false;
  }
  return true;
}

void wait_button_release_on_boot(){
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  if(digitalRead(BUTTON_PIN)==LOW)
    while(digitalRead(BUTTON_PIN)==LOW) delay(10);
}