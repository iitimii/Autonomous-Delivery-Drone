#include "actuators/motors.hpp"
#include "actuators/outputs.hpp"
#include "uav/drone.hpp"
#include "communication/telemetry.hpp"
#include "communication/receiver.hpp"
#include "uav/led.hpp"
#include "attitude_estimation/attitude_estimation.hpp"

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

        outputs::throttle = constrain(outputs::throttle, initial_speed, outputs::max_throtle);

        fr = outputs::throttle - outputs::pitch - outputs::roll - outputs::yaw; // FR CCW 1
        fl = outputs::throttle - outputs::pitch + outputs::roll + outputs::yaw; // FL CW 4
        br = outputs::throttle + outputs::pitch - outputs::roll + outputs::yaw; // BR CW 2
        bl = outputs::throttle + outputs::pitch + outputs::roll - outputs::yaw; // BL CCW 3

        // Clamp motor speeds within allowable range
        fr = constrain(fr, idle_speed, max_speed);
        fl = constrain(fl, idle_speed, max_speed);
        br = constrain(br, idle_speed, max_speed);
        bl = constrain(bl, idle_speed, max_speed);
    }

    void off()
    {
        fr = initial_speed;
        fl = initial_speed;
        br = initial_speed;
        bl = initial_speed;
    }

    void idle()
    {
        fr = idle_speed;
        fl = idle_speed;
        br = idle_speed;
        bl = idle_speed;
    }

    void balance_props()
    {
        int32_t vibration_array[20], avarage_vibration_level, vibration_total_result, vibration_total_result_final;
        uint8_t array_counter, throttle_init_ok = 0, vibration_counter;
        uint32_t wait_timer;
        led::off();

        while (1)
        {
            drone::loop();

            if (throttle_init_ok)
            {
                led::on();
                attitude_estimator.update();

                vibration_array[0] = attitude_estimator.acc_resultant;

                for (array_counter = 16; array_counter > 0; array_counter--)
                {
                    vibration_array[array_counter] = vibration_array[array_counter - 1];
                    avarage_vibration_level += vibration_array[array_counter];
                }
                avarage_vibration_level /= 17;

                if (vibration_counter < 20)
                {
                    vibration_counter++;
                    vibration_total_result += abs(vibration_array[0] - avarage_vibration_level);
                }
                else
                {
                    vibration_counter = 0;
                    vibration_total_result_final = vibration_total_result / 50;
                    vibration_total_result = 0;
                }

                if (receiver::switch_a > 1500)
                {
                    fr = 1000; // 65
                    fl = 1000; // 55
                    br = 1000; // 60
                    bl = 1000; // 50 perfect vibration

                    switch (receiver::vra)
                    {
                    case 990 ... 1050:
                        fr = receiver::throttle;
                        break;

                    case 1200 ... 1400:
                        fl = receiver::throttle;
                        break;

                    case 1480 ... 1700:
                        br = receiver::throttle;
                        break;

                    case 1800 ... 2100:
                        bl = receiver::throttle;
                        break;

                    default:
                        fr = 1000;
                        fl = 1000;
                        br = 1000;
                        bl = 1000;
                        break;
                    }

                    set_speed();
                }

                else
                    off();
            }

            else
            {
                led::off();
                wait_timer = millis() + 10000;
                while (wait_timer > millis() && !throttle_init_ok)
                {
                    if (receiver::throttle < 1050 && receiver::throttle > 990)
                        throttle_init_ok = 1;
                    delay(500);
                }
            }

            attitude_estimator.acc_resultant = vibration_total_result_final;

            telemetry::send();
            drone::wait();
        }
    }
}
