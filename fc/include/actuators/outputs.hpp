#ifndef OUTPUTS_HPP
#define OUTPUTS_HPP
#include <cstdint>

namespace outputs
{
        extern int16_t pitch, roll, throttle, yaw;

        struct Angle
        {
            int16_t pitch, roll, yaw;
        };

        extern Angle angle;

        extern int16_t max_throtle;

}

#endif // !OUTPUTS_HPP