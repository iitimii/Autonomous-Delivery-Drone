#ifndef DEBUGGING
#define DEBUGGING

#include <Arduino.h>

namespace debugging
{
    extern bool isDebug;

    void setup(const bool &debug);
    void log(const String &message);
    void print_address(uint8_t address);
} // namespace debugging

#endif