#pragma once
#include <Arduino.h>

extern bool g_debugMode;

static inline uint32_t log_millis(){ return millis(); }
#define LOG_ENABLED() (g_debugMode)
#define LOG_BASE(L,F,...) do{ if(LOG_ENABLED()) Serial.printf("[%s %10lu] " F "\n",L,(unsigned long)log_millis(),##__VA_ARGS__);}while(0)
#define LOG_INFO(F,...)  LOG_BASE("INFO ",F,##__VA_ARGS__)
#define LOG_WARN(F,...)  LOG_BASE("WARN ",F,##__VA_ARGS__)
#define LOG_FATAL(F,...) LOG_BASE("FATAL",F,##__VA_ARGS__)