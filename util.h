#pragma once
#include <stdint.h>

inline void put_u16_be(uint8_t* p,uint16_t v){ p[0]=(uint8_t)(v>>8); p[1]=(uint8_t)v; }
inline void put_u32_be(uint8_t* p,uint32_t v){ p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v; }
uint32_t get_u32_be(const uint8_t* p);
uint16_t get_u16_be(const uint8_t* p);