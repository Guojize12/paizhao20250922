#include "util.h"

uint32_t get_u32_be(const uint8_t* p){ return (uint32_t)p[0]<<24 | (uint32_t)p[1]<<16 | (uint32_t)p[2]<<8 | p[3]; }
uint16_t get_u16_be(const uint8_t* p){ return (uint16_t)p[0]<<8 | p[1]; }