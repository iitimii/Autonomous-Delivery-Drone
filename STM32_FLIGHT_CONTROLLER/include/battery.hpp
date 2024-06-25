#ifndef BATTERY_HPP
#define BATTERY_HPP

namespace battery
{
    int pin;
    int adc_res;
    float voltage;
    
    void setup();
    void read();
}

#endif // !BATTERY_HPP