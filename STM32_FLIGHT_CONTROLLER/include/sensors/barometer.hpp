#ifndef BARO_HPP
#define BARO_HPP
#include <cstdint>

namespace Sensors
{

class Barometer
{
    private:
    float pressure;
    float temperature;

    public:
    Barometer();
    float get_pressure(){return 0.01f;};
    float get_temperature(){return 0.01f;};

};

extern Barometer barometer;

}

#endif // !BARO_HPP