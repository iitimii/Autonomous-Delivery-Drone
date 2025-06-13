#include "actuators/motors.hpp"
#include "actuators/outputs.hpp"
#include "uav/drone.hpp"
#include "communication/telemetry.hpp"
#include "communication/receiver.hpp"
#include "uav/led.hpp"
#include "state_estimation/state_estimation.hpp"

namespace motors
{
    // PWM Configuration
    const int freq = 50;               // 50 Hz PWM frequency for ESCs
    const int resolution = 16;         // 16-bit resolution (0-65535)
    
    // Pin assignments (using your specified pins)
    const int ch1_pin = 27;            // FR motor
    const int ch2_pin = 12;            // FL motor
    const int ch3_pin = 13;            // BR motor
    const int ch4_pin = 14;            // BL motor
    
    // LEDC channel assignments
    const int ch1_channel = 0;         // LEDC channel for FR
    const int ch2_channel = 1;         // LEDC channel for FL
    const int ch3_channel = 2;         // LEDC channel for BR
    const int ch4_channel = 4;         // LEDC channel for BL
    
    // Speed constants
    constexpr int16_t initial_speed = 1000;
    constexpr int16_t idle_speed = 1100;
    constexpr int16_t max_speed = 1995;
    
    int16_t fr = initial_speed, fl = initial_speed, br = initial_speed, bl = initial_speed;
    
    uint32_t microsecondsToDuty(int us) {
        // Convert microseconds (1000-2000) to duty cycle (3277-6554 for 16-bit resolution)
        // Standard ESC range is 1000-2000 μs, which is 5-10% duty cycle at 50Hz
        return map(us, 1000, 2000, 3277, 6554);
    }

    void setup()
    {
        // Configure LEDC channels for PWM generation
        ledcSetup(ch1_channel, freq, resolution);
        ledcSetup(ch2_channel, freq, resolution);
        ledcSetup(ch3_channel, freq, resolution);
        ledcSetup(ch4_channel, freq, resolution);
        
        // Attach pins to channels
        ledcAttachPin(ch1_pin, ch1_channel);
        ledcAttachPin(ch2_pin, ch2_channel);
        ledcAttachPin(ch3_pin, ch3_channel);
        ledcAttachPin(ch4_pin, ch4_channel);
        
        // Initialize all motors to minimum speed
        ledcWrite(ch1_channel, microsecondsToDuty(initial_speed));
        ledcWrite(ch2_channel, microsecondsToDuty(initial_speed));
        ledcWrite(ch3_channel, microsecondsToDuty(initial_speed));
        ledcWrite(ch4_channel, microsecondsToDuty(initial_speed));
    }

    void set_speed()
    {
        // Set motor speeds using LEDC
        ledcWrite(ch1_channel, microsecondsToDuty(fr)); // FR
        ledcWrite(ch2_channel, microsecondsToDuty(fl)); // FL
        ledcWrite(ch3_channel, microsecondsToDuty(br)); // BR
        ledcWrite(ch4_channel, microsecondsToDuty(bl)); // BL
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
}