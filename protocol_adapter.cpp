#include "protocol_adapter.h"

static uint32_t s_last_tx_ms = 0;
static volatile uint32_t s_tx_frames=0;
static volatile uint32_t s_rx_frames=0;
static volatile uint32_t s_rx_crc_err=0;
static volatile uint32_t s_rx_discard=0;
static volatile uint8_t  s_busy=0;

void platform_send_packet(char op, uint16_t cmd, uint8_t pid,
                          const uint8_t* payload, uint32_t payloadLen){
  // 发送节流：最小发送间隔（不改协议/字节流）
  if(PROTO_MIN_SEND_INTERVAL_MS > 0){
    uint32_t now = millis();
    uint32_t dt = now - s_last_tx_ms;
    if(dt < PROTO_MIN_SEND_INTERVAL_MS){
      delay(PROTO_MIN_SEND_INTERVAL_MS - dt);
    }
  }
  s_busy = 1;
  platform_send_packet_impl(op, cmd, pid, payload, payloadLen);
  s_busy = 0;
  s_tx_frames++;
  s_last_tx_ms = millis();
}

void platform_serial_poll(){
  // 解析轮询仍然调用你的“原实现”
  platform_serial_poll_impl();
}

// 供解析状态机调用的统计钩子
extern "C" void proto_adapter_on_crc_err(){ s_rx_crc_err++; }
extern "C" void proto_adapter_on_discard(){ s_rx_discard++; }
extern "C" void proto_adapter_on_rx_ok(){ s_rx_frames++; }

// 计数导出
extern "C" uint32_t proto_get_rx_crc_err(){ return s_rx_crc_err; }
extern "C" uint32_t proto_get_rx_discard(){ return s_rx_discard; }
extern "C" uint32_t proto_get_rx_frames(){ return s_rx_frames; }
extern "C" uint32_t proto_get_tx_frames(){ return s_tx_frames; }
extern "C" uint8_t  proto_get_busy(){ return s_busy; }