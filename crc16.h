#pragma once
#include <stdint.h>
#include <stddef.h>

uint16_t crc16_modbus(const uint8_t* data,size_t len);

struct CRC16ModbusCtx{
  uint8_t hi{0xFF}, lo{0xFF};
  void update(const uint8_t* d,size_t n);
  uint16_t value()const{ return (uint16_t(hi)<<8)|lo; }
};