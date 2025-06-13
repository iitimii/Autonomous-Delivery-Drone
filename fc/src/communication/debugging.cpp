#include "communication/debugging.hpp"

bool debugging::isDebug = false;

void debugging::setup(const bool &debug)
{
    isDebug = debug;
    if (isDebug)
        Serial.begin(115200);
}

void debugging::log(const String &message)
{
    if (isDebug)
        Serial.println(message);
}

void debugging::print_address(uint8_t address)
{
    Serial.print(address, HEX);
}