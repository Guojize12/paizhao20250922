#pragma once
#include <Arduino.h>

void print_help();
void print_version();
void print_modes();
void sendModeLine();
void handle_serial_ascii_commands();