#include "motors.hpp"

namespace motors
{
    constexpr int ch1_out = PB6;
    constexpr int ch2_out = PB7;
    constexpr int ch3_out = PB8;
    constexpr int ch4_out = PB9;

    constexpr int16_t initial_speed = 1000;
    constexpr int16_t idle_speed = 1100;
    constexpr int16_t max_speed = 1995;

    TIM_TypeDef *Instance_out = TIM4;
    HardwareTimer *OutTim = new HardwareTimer(Instance_out);

    int16_t fr = initial_speed, fl = initial_speed, br = initial_speed, bl = initial_speed;

    void setup()
    {
        OutTim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, ch1_out);
        OutTim->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, ch2_out);
        OutTim->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, ch3_out);
        OutTim->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, ch4_out);
        OutTim->setPrescaleFactor(84);
        OutTim->setOverflow(5001, MICROSEC_FORMAT);

        for (int channel = 1; channel <= 4; ++channel)
        {
            OutTim->setCaptureCompare(channel, initial_speed, MICROSEC_COMPARE_FORMAT);
        }

        OutTim->resume();
    }

    void set_speed()
    {
        // Set motor speeds
        OutTim->setCaptureCompare(1, fr, MICROSEC_COMPARE_FORMAT); // PB6 FR
        OutTim->setCaptureCompare(2, fl, MICROSEC_COMPARE_FORMAT); // PB7 FL
        OutTim->setCaptureCompare(3, br, MICROSEC_COMPARE_FORMAT); // PB8 BR
        OutTim->setCaptureCompare(4, bl, MICROSEC_COMPARE_FORMAT); // PB9 BL
        OutTim->setCount(5000, MICROSEC_FORMAT);                   // Set timer count
    }

    void running()
    {
        // Clamp motor speeds within allowable range
        fr = constrain(fr, idle_speed, max_speed);
        fl = constrain(fl, idle_speed, max_speed);
        br = constrain(br, idle_speed, max_speed);
        bl = constrain(bl, idle_speed, max_speed);
        set_speed();
    }

    void off()
    {
        fr = initial_speed;
        fl = initial_speed;
        br = initial_speed;
        bl = initial_speed;
        set_speed();
    }

    void idle()
    {
        fr = idle_speed;
        fl = idle_speed;
        br = idle_speed;
        bl = idle_speed;
        set_speed();
    }
}
