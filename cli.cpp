#include "cli.h"
#include "logging.h"
#include "config.h"
#include "cam_sd.h"
#include "protocol.h"
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>
#include "protocol_adapter.h" 

extern uint8_t g_last_rx_packet[];
extern size_t  g_last_rx_packet_len;
extern char g_sim_iccid[21];
extern bool g_debugMode;

static bool readNumberImmediate(int &value){
  uint32_t start=millis(); value=-1; bool any=false;
  while(Serial.available() || (millis()-start)<20){
    while(Serial.available()){
      int c=Serial.peek();
      if(c>='0' && c<='9'){
        Serial.read();
        if(!any){ value=0; any=true; }
        value=value*10 + (c-'0');
      }else{
        Serial.read(); return any;
      }
    }
    if(any) break;
    delay(1);
  }
  return any;
}

void print_help(){
  if(!g_debugMode) return;
  Serial.println("命令:");
  Serial.println("  C=拍照 I=信息 J=JSON H=帮助 V=版本 G=模式");
  Serial.println("  bNN=闪光占空 s=保存参数 R=重挂SD Mx模式位 Qn质量 Fn分辨率 P重建相机");
  Serial.println("  d/w 切换模式 K=模式 Z=CRC测试");
  Serial.println("  :H 心跳 :T 时间请求 :P 开机状态 :S SIM :A 协议状态 :E 协议统计 :X 最近平台包HEX");
  Serial.println("  获取图片: 平台 CMD=0x8015");
#if UPGRADE_ENABLE
  Serial.println("  升级: 平台 0x8005 -> 0x0003/0x0004 OTA");
#endif
}

void print_version(){
  if(!g_debugMode) return;
  Serial.printf("[VER] Build %s %s Pref=%d Q=%d\n",
    __DATE__,__TIME__,(int)FRAME_SIZE_PREF,(int)JPEG_QUALITY_PREF);
}

void print_modes(){
  if(!g_debugMode) return;
  Serial.printf("[MODE] sendBeforeSave=%d save=%d send=%d debug=%d\n",
    g_cfg.sendBeforeSave,g_cfg.saveEnabled,g_cfg.sendEnabled,(int)g_debugMode);
}

void sendModeLine(){ Serial.printf("#MODE {\"debug\":%d}\n",(int)g_debugMode); }

void handle_serial_ascii_commands(){
  while(Serial.available()){
    int pk=Serial.peek();
    if(pk=='$') return;  // 让协议解析器处理二进制协议帧
    char c=Serial.read();
    if(c=='\r'||c=='\n') continue;

    // 冒号命令（双字符）
    if(c==':'){
      while(!Serial.available()) delay(0);
      char s=Serial.read();
      switch(s){
        case 'H': case 'h': platform_send_heartbeat(); break;
        case 'T': case 't': platform_send_time_request(); break;
        case 'P': case 'p': platform_send_power_on_status(); break;
        case 'S': case 's': platform_send_sim_info(); break;
        case 'A': case 'a': print_protocol_status(); break;
        case 'E': case 'e': { // 新增：协议统计（来自 protocol_adapter）
          Serial.printf("rx_ok=%u crc_err=%u discard=%u tx=%u busy=%u\n",
            proto_get_rx_frames(), proto_get_rx_crc_err(), proto_get_rx_discard(),
            proto_get_tx_frames(), proto_get_busy());
        } break;
        case 'X': case 'x':
          if(g_debugMode){
            Serial.println("[RX-HEX] 请在协议模块内打印或按需导出该缓冲区。");
          }
          break;
        default:
          if(g_debugMode) Serial.printf("[CMD] unknown :%c\n",s);
      }
      continue;
    }

    // 单字符即时命令
    switch(c){
      case 'C': case 'c': capture_and_process(TRIGGER_BUTTON); break;
      case 'I': case 'i':
        if(g_debugMode){
          uint64_t sd_total=(SD.cardType()==CARD_NONE)?0:SD.totalBytes()/(1024*1024);
          LOG_INFO("photos=%u failed=%u sd_total=%lluMB",
            g_stats.total_captures,g_stats.failed_captures,sd_total);
        }
        break;
      case 'J': case 'j':
        if(g_debugMode)
          Serial.printf("{\"photo_idx\":%u,\"total\":%u}\n",0u,g_stats.total_captures);
        break;
      case 'H': case 'h': print_help(); break;
      case 'V': case 'v': print_version(); break;
      case 'G': case 'g': print_modes(); break;
      case 'K': case 'k': sendModeLine(); break;
      case 'd': case 'D': g_debugMode=true; sendModeLine(); print_help(); break;
      case 'w': case 'W': g_debugMode=false; sendModeLine(); break;
      case 'b': case 'B': {
        int v; if(readNumberImmediate(v)){ if(v<0)v=0; if(v>255)v=255; flashSet(v); }
      } break;
      case 's': case 'S': save_params_to_nvs(); break;
      case 'R': case 'r': init_sd(); break;
      case 'M': case 'm': {
        int bit; if(!readNumberImmediate(bit)){ if(g_debugMode) Serial.println("M<0-2>"); break; }
        switch(bit){
          case 0: g_cfg.sendBeforeSave=!g_cfg.sendBeforeSave; break;
          case 1: g_cfg.saveEnabled=!g_cfg.saveEnabled; break;
          case 2: g_cfg.sendEnabled=!g_cfg.sendEnabled; break;
          default: if(g_debugMode) Serial.println("未知位");
        }
        print_modes();
      } break;
      case 'Q': case 'q': {
        int q; if(!readNumberImmediate(q)){ if(g_debugMode) Serial.println("Q<5-63>"); break; }
        if(q<5)q=5; if(q>63)q=63;
        if(camera_ok){
          sensor_t* s=esp_camera_sensor_get();
          if(s){ s->set_quality(s,q); if(g_debugMode) Serial.printf("[CAM] quality=%d\n",q); }
        }
      } break;
      case 'F': case 'f': {
        int fs; if(!readNumberImmediate(fs)){ if(g_debugMode) Serial.println("F<0-13>"); break; }
        if(fs<0 || fs>FRAMESIZE_UXGA){ if(g_debugMode) Serial.println("无效"); break; }
        reinit_camera_with_params((framesize_t)fs,JPEG_QUALITY_PREF);
      } break;
      case 'P': case 'p':
        reinit_camera_with_params(FRAME_SIZE_PREF,JPEG_QUALITY_PREF); break;
      case 'Z': case 'z': {
        const char* s="123456789";
        uint16_t cval=0;
        // 提示：CRC 测试请在协议模块实现，如需暴露可添加接口
        Serial.printf("#CRCTEST {\"crc\":0x%04X}\n",cval);
      } break;
      default:
        if(g_debugMode) Serial.printf("[CMD] unknown '%c'\n",c);
    }
  }
}