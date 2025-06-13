#include "uav/eeprom.hpp"


namespace eeprom
{
    void setup()
    {
        EEPROM.begin(EEPROM_SIZE);
    }
} // namespace eeprom