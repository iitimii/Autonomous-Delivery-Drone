#ifndef EEPROM_HPP
#define EEPROM_HPP
#define EEPROM_SIZE 24

#include <EEPROM.h>

namespace eeprom
{
    void setup();

    template <typename T>
    void write(int address, const T &value)
    {
        EEPROM.put(address, value);
        EEPROM.commit();
    }

    template <typename T>
    T read(int address, const T &value)
    {
        T result;
        EEPROM.get(address, result);
        return result;
    }
}


#endif // !EEPROM_HPP