#include "protocol.h"
#include "logging.h"
#include "util.h"
#include "crc16.h"
#include <SPI.h>
#include <SD.h>
#include "esp_camera.h"
#include <string.h>
#include "protocol_adapter.h" 
#include "config.h"

#if UPGRADE_ENABLE
#include "upgrade.h"
#endif

// 全局/状态
uint32_t g_normal_interval_sec=10;
uint32_t g_work_interval_sec=0;
PlatformAddress g_server_addr={false,"",0};
static uint8_t g_last_rx_packet[256];
static size_t  g_last_rx_packet_len=0;

uint8_t g_pid_counter=1;
static char g_device_sn[13]="000000000000";
char g_sim_iccid[21];
uint8_t g_sim_signal_strength=0;

//bool g_debugMode = true;

// 相机状态由相机模块提供
extern bool camera_ok;
// 拍照函数由相机模块提供
extern bool capture_and_process(uint8_t trigger);

// 主动图片上传状态
uint32_t g_last_image_sent_ms=0;
PendingImage* g_pending_image=nullptr;

// 时序
uint32_t g_lastHeartbeatSentMs=0;
uint32_t g_lastTimeReqSentMs=0;
bool g_sentPowerOn=false;
bool g_sentSimInfo=false;

// 串口解析缓存
static uint8_t g_proto_rx_buf[512];
static size_t  g_proto_rx_len=0;

// 构包/发送
size_t platform_build_header(char opType,uint16_t cmd,uint8_t pid,uint16_t payloadLen,uint8_t* out){
  out[0]='$'; out[1]=(uint8_t)opType;
  out[2]=(uint8_t)(payloadLen>>8); out[3]=(uint8_t)payloadLen;
  for(int i=0;i<12;i++) out[4+i]=(uint8_t)g_device_sn[i];
  out[16]=PLATFORM_VER;
  out[17]=(uint8_t)(cmd>>8);
  out[18]=(uint8_t)cmd;
  out[19]=PLATFORM_DMODEL;
  out[20]=pid;
  uint16_t crc=crc16_modbus(out,21);
  out[21]=(uint8_t)(crc & 0xFF);
  out[22]=(uint8_t)(crc>>8);
  return 23;
}

#define SMALL_PACKET_MAX 480
static void platform_send_packet_small(char op,uint16_t cmd,uint8_t pid,const uint8_t* payload,uint16_t payloadLen){
  uint8_t out[512];
  size_t off=platform_build_header(op,cmd,pid,payloadLen,out);
  if(payloadLen){
    memcpy(out+off,payload,payloadLen);
    uint16_t dcrc=crc16_modbus(out+off,payloadLen);
    out[off+payloadLen]=(uint8_t)(dcrc & 0xFF);
    out[off+payloadLen+1]=(uint8_t)(dcrc>>8);
    off+=payloadLen+2;
  }
  Serial.write(out,off); Serial.flush();
  if(g_debugMode) LOG_INFO("[TX] cmd=0x%04X pid=%u len=%u total=%u",cmd,pid,payloadLen,(unsigned)off);
}
static void platform_send_packet_large(char op,uint16_t cmd,uint8_t pid,const uint8_t* payload,uint32_t payloadLen){
  uint8_t header[23];
  platform_build_header(op,cmd,pid,(uint16_t)payloadLen,header);
  Serial.write(header,23);
  CRC16ModbusCtx ctx;
  const size_t CHUNK=1024;
  uint32_t sent=0;
  while(sent<payloadLen){
    size_t n=payloadLen-sent; if(n>CHUNK) n=CHUNK;
    Serial.write(payload+sent,n);
    ctx.update(payload+sent,n);
    sent+=n;
  }
  uint16_t crc=ctx.value();
  uint8_t tail[2]={(uint8_t)(crc & 0xFF),(uint8_t)(crc>>8)};
  Serial.write(tail,2); Serial.flush();
  if(g_debugMode) LOG_INFO("[TX-L] cmd=0x%04X pid=%u payload=%lu",cmd,pid,(unsigned long)payloadLen);
}

// 改名为 _impl：真实发送实现（对外导出的 platform_send_packet 由适配层提供）
void platform_send_packet_impl(char op,uint16_t cmd,uint8_t pid,const uint8_t* payload,uint32_t payloadLen){
  if(payloadLen<=SMALL_PACKET_MAX) platform_send_packet_small(op,cmd,pid,payload,(uint16_t)payloadLen);
  else platform_send_packet_large(op,cmd,pid,payload,payloadLen);
}

void platform_send_ack_W(uint16_t cmd,uint8_t pid,uint8_t status){
  uint8_t p[1]={status}; platform_send_packet('W',cmd,pid,p,1);
}
void platform_send_ack_R(uint16_t cmd,uint8_t pid,uint8_t status){
  uint8_t p[1]={status}; platform_send_packet('R',cmd,pid,p,1);
}

// 上行
void platform_send_heartbeat(){ platform_send_packet('R',CMD_HEARTBEAT_REQ,0,nullptr,0); g_lastHeartbeatSentMs=millis();}
void platform_send_time_request(){ platform_send_packet('R',CMD_TIME_SYNC_REQ,g_pid_counter++,nullptr,0); g_lastTimeReqSentMs=millis();}
void platform_send_power_on_status(){
  uint8_t payload[37];
  uint16_t year=2025; uint8_t month=1,day=1,hour=0,minute=0,second=(uint8_t)((millis()/1000)%60);
  put_u16_be(payload+0,year);
  payload[2]=month;payload[3]=day;payload[4]=hour;payload[5]=minute;payload[6]=second;
  payload[7]=payload[8]=payload[9]=payload[10]=0;
  payload[11]=1;payload[12]=0;payload[13]=0;
  payload[14]=1;payload[15]=0;payload[16]=0;
  memset(payload+17,0,20);
  const char model[]="CAM-ESP32";
  memcpy(payload+17,model,strlen(model)>20?20:strlen(model));
  platform_send_packet('R',CMD_POWER_ON_STATUS,g_pid_counter++,payload,37);
  g_sentPowerOn=true;
}
void platform_send_sim_info(){
  uint8_t payload[30];
  uint16_t year=2025; uint8_t month=1,day=1,hour=0,minute=0,second=(uint8_t)((millis()/1000)%60);
  put_u16_be(payload+0,year);
  payload[2]=month;payload[3]=day;payload[4]=hour;payload[5]=minute;payload[6]=second;
  payload[7]=20; memset(payload+8,'0',20);
  payload[28]=0; payload[29]=0;
  platform_send_packet('R',CMD_SIM_INFO,g_pid_counter++,payload,30);
  g_sentSimInfo=true;
}

void platform_apply_time(const uint8_t* d,uint16_t len){
  if(len<7) return;
  uint16_t year=get_u16_be(d);
  if(g_debugMode) LOG_INFO("[TIME] %04u-%02u-%02u %02u:%02u:%02u",year,d[2],d[3],d[4],d[5],d[6]);
  // TODO: 设置 RTC
}

uint8_t platform_set_intervals(const uint8_t* d,uint16_t len){
  if(len<8) return RESP_PARSE_FAIL;
  uint32_t normal=get_u32_be(d);
  uint32_t work=get_u32_be(d+4);
  if(normal<NORMAL_INTERVAL_MIN_SEC) normal=NORMAL_INTERVAL_MIN_SEC;
  if(normal>NORMAL_INTERVAL_MAX_SEC) normal=NORMAL_INTERVAL_MAX_SEC;
  if(work>0){
    if(work<WORK_INTERVAL_MIN_SEC) work=WORK_INTERVAL_MIN_SEC;
    if(work>WORK_INTERVAL_MAX_SEC) work=WORK_INTERVAL_MAX_SEC;
  }
  g_normal_interval_sec=normal;
  g_work_interval_sec=work;
  if(g_debugMode) LOG_INFO("[INT] normal=%lu work=%lu",(unsigned long)normal,(unsigned long)work);
  return RESP_OK;
}

uint8_t platform_set_address(const uint8_t* d,uint16_t len){
  if(len<9) return RESP_PARSE_FAIL;
  uint8_t pos=d[0];
  if(pos!=1) return RESP_UNSUPPORT;
  uint32_t hostLen=get_u32_be(d+1);
  uint32_t port=get_u32_be(d+5);
  if(9+hostLen>len) return RESP_PARSE_FAIL;
  if(hostLen>=sizeof(g_server_addr.host)) hostLen=sizeof(g_server_addr.host)-1;
  char tmp[128]; memset(tmp,0,sizeof(tmp));
  memcpy(tmp,d+9,hostLen);
  if(hostLen==0 || strcmp(tmp,"0.0.0.0")==0){
    g_server_addr.valid=false; g_server_addr.host[0]='\0'; g_server_addr.port=0;
    if(g_debugMode) LOG_INFO("[ADDR] cleared");
    return RESP_OK;
  }
  g_server_addr.valid=true;
  memcpy(g_server_addr.host,tmp,hostLen+1);
  g_server_addr.port=port;
  if(g_debugMode) LOG_INFO("[ADDR] host=%s port=%lu",g_server_addr.host,(unsigned long)port);
  return RESP_OK;
}

void platform_get_address(uint8_t pid,const uint8_t* d,uint16_t len){
  if(len<1){ platform_send_ack_W(CMD_S_GET_ADDR,pid,RESP_PARSE_FAIL); return; }
  uint8_t pos=d[0];
  if(pos!=1){ platform_send_ack_W(CMD_S_GET_ADDR,pid,RESP_UNSUPPORT); return; }
  uint8_t payload[256]; size_t off=0;
  if(!g_server_addr.valid){
    payload[off++]=RESP_OK; payload[off++]=pos;
    put_u32_be(payload+off,0); off+=4; put_u32_be(payload+off,0); off+=4;
  }else{
    size_t hl=strlen(g_server_addr.host); if(hl>120) hl=120;
    payload[off++]=RESP_OK; payload[off++]=pos;
    put_u32_be(payload+off,(uint32_t)hl); off+=4;
    put_u32_be(payload+off,g_server_addr.port); off+=4;
    memcpy(payload+off,g_server_addr.host,hl); off+=hl;
  }
  platform_send_packet('W',CMD_S_GET_ADDR,pid,payload,(uint16_t)off);
}

// 主动图片上传
static bool image_can_send_now(){
  uint32_t now=millis();
  if(now - g_last_image_sent_ms < IMAGE_MIN_INTERVAL_MS){
    if(g_debugMode) LOG_WARN("[IMG] interval limit"); return false;
  }
  if(g_pending_image){ if(g_debugMode) LOG_WARN("[IMG] still pending"); return false; }
#if UPGRADE_ENABLE
  if(g_upg.state==UPG_DOWNLOADING || g_upg.state==UPG_FILE_INFO){
    if(g_debugMode) LOG_WARN("[IMG] blocked by upgrade");
    return false;
  }
#endif
  return true;
}

void platform_send_image(uint8_t trigger,const uint8_t* jpeg,uint32_t jpegLen,uint32_t frameIndex){
  if(jpegLen>IMAGE_MAX_LEN){ LOG_WARN("[IMG] too large %lu",(unsigned long)jpegLen); return; }
  if(!image_can_send_now()) return;
  uint32_t payloadLen=IMAGE_META_LEN + jpegLen;
  uint8_t* buf=(uint8_t*)malloc(payloadLen);
  if(!buf){ LOG_WARN("[IMG] malloc fail"); return; }
  uint16_t year=2025; uint8_t month=1,day=1,hour=0,minute=0,second=(uint8_t)((millis()/1000)%60);
  put_u16_be(buf+0,year);
  buf[2]=month;buf[3]=day;buf[4]=hour;buf[5]=minute;buf[6]=second;
  buf[7]=trigger;
  put_u32_be(buf+8,PLATFORM_SLAVE_ID);
  put_u32_be(buf+12,jpegLen);
  memcpy(buf+16,jpeg,jpegLen);

  PendingImage* pi=(PendingImage*)malloc(sizeof(PendingImage));
  if(!pi){ free(buf); LOG_WARN("[IMG] struct alloc fail"); return; }
  uint8_t pid=g_pid_counter++;
  pi->payload=buf; pi->payloadLen=payloadLen; pi->imageLen=jpegLen;
  pi->pid=pid; pi->trigger=trigger; pi->retries_left=IMAGE_MAX_RETRIES;
  pi->last_send_ms=0; pi->frame_index=frameIndex;
  g_pending_image=pi;

  platform_send_packet('R',CMD_IMAGE_UPLOAD,pid,buf,payloadLen);
  g_pending_image->last_send_ms=millis();
  g_last_image_sent_ms=g_pending_image->last_send_ms;
  if(g_debugMode) LOG_INFO("[IMG] sent frame=%lu len=%lu pid=%u trig=%u",
    (unsigned long)frameIndex,(unsigned long)jpegLen,pid,trigger);
}

void image_retry_scheduler(){
  if(!g_pending_image) return;
  uint32_t now=millis();
  if(now - g_pending_image->last_send_ms >= IMAGE_ACK_TIMEOUT_MS){
    if(g_pending_image->retries_left>0){
      g_pending_image->retries_left--;
      platform_send_packet('R',CMD_IMAGE_UPLOAD,g_pending_image->pid,
                           g_pending_image->payload,g_pending_image->payloadLen);
      g_pending_image->last_send_ms=now;
      if(g_debugMode) LOG_WARN("[IMG] resend frame=%lu left=%u",
        (unsigned long)g_pending_image->frame_index,g_pending_image->retries_left);
    }else{
      if(g_debugMode) LOG_WARN("[IMG] give up frame=%lu",
        (unsigned long)g_pending_image->frame_index);
      free(g_pending_image->payload); free(g_pending_image); g_pending_image=nullptr;
    }
  }
}

void image_handle_ack(uint8_t pid,const uint8_t* payload,uint16_t len){
  if(!g_pending_image || pid!=g_pending_image->pid || len!=1) return;
  if(g_debugMode) LOG_INFO("[IMG] ACK pid=%u status=0x%02X",pid,payload[0]);
  free(g_pending_image->payload); free(g_pending_image); g_pending_image=nullptr;
}

// 平台拉取图片
void platform_handle_get_image_request(const uint8_t* payload, uint16_t len, uint8_t pid){
  if(len<2){
    uint8_t resp[1]={RESP_PARSE_FAIL};
    platform_send_packet('W',CMD_S_GET_IMAGE,pid,resp,1);
    return;
  }
  uint8_t pos=payload[0], attr=payload[1];
  bool supportedPos = (pos==1 || pos==5);
#if UPGRADE_ENABLE
  if(g_upg.state==UPG_DOWNLOADING || g_upg.state==UPG_FILE_INFO) supportedPos=false;
#endif
  if(g_pending_image) supportedPos=false;
  if(!supportedPos){
    uint8_t resp[1]={RESP_FAIL};
    platform_send_packet('W',CMD_S_GET_IMAGE,pid,resp,1);
    return;
  }
  if(attr!=1 && attr!=2){
    uint8_t resp[1]={RESP_PARSE_FAIL};
    platform_send_packet('W',CMD_S_GET_IMAGE,pid,resp,1);
    return;
  }
  if(!camera_ok){
    uint8_t resp[1]={RESP_FAIL};
    platform_send_packet('W',CMD_S_GET_IMAGE,pid,resp,1);
    return;
  }
  camera_fb_t* fb=esp_camera_fb_get();
  if(!fb){ delay(20); fb=esp_camera_fb_get(); }
  if(!fb){
    uint8_t resp[1]={RESP_FAIL};
    platform_send_packet('W',CMD_S_GET_IMAGE,pid,resp,1);
    return;
  }
  uint32_t imgLen=fb->len;
  if(imgLen>GET_IMAGE_MAX_LEN){
    esp_camera_fb_return(fb);
    uint8_t resp[1]={RESP_FAIL};
    platform_send_packet('W',CMD_S_GET_IMAGE,pid,resp,1);
    return;
  }
  uint32_t respLen=1+1+4+imgLen;
  uint8_t* buf=(uint8_t*)malloc(respLen);
  if(!buf){
    esp_camera_fb_return(fb);
    uint8_t resp[1]={RESP_FAIL};
    platform_send_packet('W',CMD_S_GET_IMAGE,pid,resp,1);
    return;
  }
  buf[0]=RESP_OK; buf[1]=pos; put_u32_be(buf+2,imgLen);
  memcpy(buf+6,fb->buf,imgLen);
  esp_camera_fb_return(fb);
  platform_send_packet('W',CMD_S_GET_IMAGE,pid,buf,respLen);
  free(buf);
  if(g_debugMode) LOG_INFO("[GET_IMG] pos=%u attr=%u len=%lu",pos,attr,(unsigned long)imgLen);
}

// 解析/分发
size_t try_parse_one_platform_packet(uint8_t* buf,size_t len){
  if(len<23) return 0;
  if(buf[0]!='$') return 1;
  uint16_t hcrc=crc16_modbus(buf,21);
  if((uint8_t)(hcrc & 0xFF)!=buf[21] || (uint8_t)(hcrc>>8)!=buf[22]){
    if(g_debugMode) LOG_WARN("[PROTO] head CRC fail");
    proto_adapter_on_crc_err();   // 新增：头CRC失败计数
    return 1;
  }
  uint16_t dlen=(uint16_t)buf[2]<<8|buf[3];
  size_t total=23 + dlen + (dlen?2:0);
  if(len<total) return 0;
  if(dlen){
    uint16_t dcrc=crc16_modbus(buf+23,dlen);
    if((uint8_t)(dcrc & 0xFF)!=buf[23+dlen] || (uint8_t)(dcrc>>8)!=buf[23+dlen+1]){
      if(g_debugMode) LOG_WARN("[PROTO] data CRC fail");
      proto_adapter_on_crc_err(); // 新增：数据CRC失败计数
      return total;
    }
  }
  size_t cp= total>sizeof(g_last_rx_packet)? sizeof(g_last_rx_packet):total;
  memcpy(g_last_rx_packet,buf,cp); g_last_rx_packet_len=cp;
  uint8_t op=buf[1];
  uint16_t cmd=(uint16_t)buf[17]<<8|buf[18];
  uint8_t pid=buf[20];
  const uint8_t* payload=dlen?buf+23:nullptr;

  if(g_debugMode) LOG_INFO("[RX] op=%c cmd=0x%04X pid=%u dlen=%u",op,cmd,pid,dlen);

  if(op=='W'){
    switch(cmd){
      case CMD_S_HEARTBEAT:
        platform_send_packet('W',CMD_S_HEARTBEAT,pid,nullptr,0); break;
      case CMD_S_TIME_SYNC:
        if(dlen>=7){ platform_apply_time(payload,dlen); platform_send_ack_W(CMD_S_TIME_SYNC,pid,RESP_OK); }
        else platform_send_ack_W(CMD_S_TIME_SYNC,pid,RESP_PARSE_FAIL);
        break;
      case CMD_S_SET_INTERVAL:{
        uint8_t st=platform_set_intervals(payload,dlen);
        platform_send_ack_W(CMD_S_SET_INTERVAL,pid,st);
      } break;
      case CMD_S_SET_ADDR:{
        uint8_t st=platform_set_address(payload,dlen);
        platform_send_ack_W(CMD_S_SET_ADDR,pid,st);
      } break;
      case CMD_S_GET_ADDR:
        platform_get_address(pid,payload,dlen); break;
      case CMD_S_PARAM_RWX:
        platform_send_ack_W(CMD_S_PARAM_RWX,pid,RESP_UNSUPPORT); break;
      case CMD_REMOTE_UPGRADE_CMD:
#if UPGRADE_ENABLE
        platform_send_ack_W(CMD_REMOTE_UPGRADE_CMD,pid,RESP_OK);
        if(g_debugMode) LOG_INFO("[UPG] trigger received");
        upgrade_set_req_and_reboot();
#else
        platform_send_ack_W(CMD_REMOTE_UPGRADE_CMD,pid,RESP_UNSUPPORT);
#endif
        break;
      case CMD_REMOTE_FUNCTION_CMD:{
        if(dlen<6){ platform_send_ack_W(CMD_REMOTE_FUNCTION_CMD,pid,RESP_PARSE_FAIL); break; }
        uint16_t funcId=get_u16_be(payload+0);
        uint32_t param=get_u32_be(payload+2); (void)param;
        uint8_t st=RESP_UNSUPPORT;
        switch(funcId){
          case 0x0001:
#if UPGRADE_ENABLE
            if(g_upg.state==UPG_DOWNLOADING||g_upg.state==UPG_FILE_INFO) st=RESP_FAIL;
            else
#endif
            { if(capture_and_process(TRIGGER_REMOTE)) st=RESP_OK; else st=RESP_FAIL; }
            break;
          case 0x0002:
            st=RESP_OK;
            platform_send_ack_W(CMD_REMOTE_FUNCTION_CMD,pid,st);
            if(g_debugMode) LOG_INFO("[FUNC] reboot");
            delay(200); ESP.restart();
            return total;
          default: st=RESP_UNSUPPORT;
        }
        platform_send_ack_W(CMD_REMOTE_FUNCTION_CMD,pid,st);
      } break;
      case CMD_S_GET_IMAGE:
        platform_handle_get_image_request(payload,dlen,pid);
        break;
      default:
        platform_send_ack_W(cmd,pid,RESP_UNSUPPORT);
    }
  }else if(op=='R'){
    if(cmd==CMD_TIME_SYNC_REQ && dlen>=7) platform_apply_time(payload,dlen);
    else if(cmd==CMD_IMAGE_UPLOAD && dlen==1) image_handle_ack(pid,payload,dlen);
#if UPGRADE_ENABLE
    else if(cmd==CMD_FILE_INFO_REQ) handle_file_info_response(payload,dlen);
    else if(cmd==CMD_FILE_BLOCK_REQ) handle_file_block_response(payload,dlen);
#endif
  }

  proto_adapter_on_rx_ok(); // 新增：成功解析一帧
  return total;
}

// 改名为 _impl：真实解析轮询（对外导出的 platform_serial_poll 由适配层提供）
void platform_serial_poll_impl(){
  while(Serial.available()){
    int ch=Serial.peek();
    if(ch=='$'){
      if(g_proto_rx_len<sizeof(g_proto_rx_buf))
        g_proto_rx_buf[g_proto_rx_len++]=(uint8_t)Serial.read();
      else { 
        // 缓冲溢出丢弃一个字节，记一次 discard
        g_proto_rx_len=0; Serial.read(); 
        proto_adapter_on_discard(); // 新增：丢弃计数
      }
      while(Serial.available() && g_proto_rx_len<sizeof(g_proto_rx_buf))
        g_proto_rx_buf[g_proto_rx_len++]=(uint8_t)Serial.read();
      size_t consumed=0;
      while(true){
        size_t c=try_parse_one_platform_packet(g_proto_rx_buf+consumed,g_proto_rx_len-consumed);
        if(c==0) break;
        consumed+=c;
        if(consumed>=g_proto_rx_len) break;
      }
      if(consumed>0 && consumed<g_proto_rx_len){
        memmove(g_proto_rx_buf,g_proto_rx_buf+consumed,g_proto_rx_len-consumed);
        g_proto_rx_len-=consumed;
      }else if(consumed==g_proto_rx_len) g_proto_rx_len=0;
    }else break;
  }
}

void print_protocol_status(){
  if(!g_debugMode) return;
  Serial.printf("[PROTO] hb=%lus work=%lus server=%d host=%s port=%lu pending_img=%s",
    (unsigned long)g_normal_interval_sec,
    (unsigned long)g_work_interval_sec,
    g_server_addr.valid,
    g_server_addr.valid?g_server_addr.host:"-",
    (unsigned long)g_server_addr.port,
    g_pending_image?"YES":"NO");
#if UPGRADE_ENABLE
  Serial.printf(" upg_state=%u recv=%lu/%lu\n",(unsigned)g_upg.state,(unsigned long)g_upg.received,(unsigned long)g_upg.totalSize);
#else
  Serial.println();
#endif
}