#include "communication/receiver.hpp"
#include "communication/telemetry.hpp"
#include "uav/drone.hpp"

// ISR function must be defined outside of namespace
volatile int32_t last_time = 0;
volatile uint8_t channel_counter = 0;
volatile uint16_t channel_values[10] = {0};

void IRAM_ATTR ppmISR() {
    // Get the current time
    int32_t current_time = micros();
    // Calculate the time difference
    int32_t delta = current_time - last_time;
    last_time = current_time;

    // Process the PPM signal
    if (delta > 2500) { // This is a sync pulse (gap between frames)
        channel_counter = 0; // Reset channel counter
    } 
    else if (channel_counter < 10) { // Valid channel pulse
        channel_values[channel_counter++] = delta;
    }
}

namespace receiver
{
    uint8_t channel_select_counter = 0;
    uint16_t channels[10] = {0};
    int32_t measured_time = 0, measured_time_start = 0;
    uint16_t &roll = channels[0];
    uint16_t &pitch = channels[1];
    uint16_t &throttle = channels[2];
    uint16_t &yaw = channels[3];
    uint16_t &switch_a = channels[4];
    uint16_t &switch_b = channels[5];
    uint16_t &switch_c = channels[6];
    uint16_t &switch_d = channels[7];
    uint16_t &vra = channels[8];
    uint16_t &vrb = channels[9];

    int pin = 34; // Default GPIO pin for PPM input - adjust as needed
    hw_timer_t *timer = NULL;
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

    void setup()
    {
        // Configure pin as input with pullup
        pinMode(pin, INPUT_PULLUP);
        
        // Attach interrupt to pin
        attachInterrupt(digitalPinToInterrupt(pin), ppmISR, RISING);
        
        // Create a timer to periodically copy volatile data to the namespace variables
        timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1MHz), count up
        timerAttachInterrupt(timer, &updateChannels, true); // Edge interrupt
        timerAlarmWrite(timer, 20000, true); // 20ms period (50Hz), auto reload
        timerAlarmEnable(timer);
    }

    // Timer ISR to update channel values
    void IRAM_ATTR updateChannels() {
        portENTER_CRITICAL_ISR(&timerMux);
        for (int i = 0; i < 10; i++) {
            channels[i] = channel_values[i];
        }
        portEXIT_CRITICAL_ISR(&timerMux);
    }

    void wait()
    {
        while (channels[0] < 990 || channels[1] < 990 || channels[2] < 800 || channels[3] < 990)
        {
            drone::error = 4;
            // telemetry::send();
            delay(4);
        }
    }
}