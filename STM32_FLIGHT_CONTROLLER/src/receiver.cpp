#include "receiver.hpp"
#include "telemetry.hpp"
#include "drone.hpp"

namespace receiver
{
     uint8_t channel_select_counter;
     uint16_t channels[10] = {0};
     int32_t measured_time, measured_time_start;

     int pin = PB10;
     TIM_TypeDef *Instance_in = TIM2;
     HardwareTimer *InTim = new HardwareTimer(Instance_in);

     uint32_t chx = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));

     void setup()
     {
          // Input from RX (TIM2)
          InTim->setMode(chx, TIMER_INPUT_CAPTURE_RISING, pin);
          InTim->setPrescaleFactor(84);
          InTim->setOverflow(0x10000, MICROSEC_FORMAT);
          InTim->setCaptureCompare(1, 1000, MICROSEC_COMPARE_FORMAT);
          InTim->attachInterrupt(chx, handler);
          InTim->resume();
     }

     void handler()
     {
          int32_t current_time = InTim->getCaptureCompare(chx, MICROSEC_COMPARE_FORMAT);
          measured_time = current_time - measured_time_start;
          if (measured_time < 0)
               measured_time += 0xFFFF;
          measured_time -= 25;
          measured_time_start = current_time;

          if (measured_time > 3000)
          {
               channel_select_counter = 0;
          }
          else
          {
               if (channel_select_counter < 10)
               {
                    channels[channel_select_counter] = measured_time;
               }
               ++channel_select_counter;
          }
     }

     void wait()
     {
          while (channels[0] < 990 || channels[1] < 990 || channels[2] < 800 || channels[3] < 990)
          {
               drone::error = 4;
               telemetry::send();
               delay(4);
          }
     }
}
