#include <Arduino.h>

TIM_TypeDef *Instance_in = TIM1;
TIM_TypeDef *Instance_out = TIM4;

HardwareTimer *InTim = new HardwareTimer(Instance_in);
HardwareTimer *OutTim = new HardwareTimer(Instance_out);

#define ch1 PA8
#define ch2 PA9
#define ch3 PA10
#define ch4 PA11
uint32_t chx = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(ch1), PinMap_PWM));

#define ch1_out PB6
#define ch2_out PB7
#define ch3_out PB8
#define ch4_out PB9


uint32_t channel1, channel2, channel3, channel4, channel5, channel6;
int32_t cur;
int32_t prev;
uint32_t loop_timer;
uint8_t channel_select_counter, error, start, receiver_watchdog;


void handler(void) {
  cur = InTim->getCaptureCompare(chx, MICROSEC_COMPARE_FORMAT) - prev;
  if (cur < 0) cur += 0x10000;
  prev = InTim->getCaptureCompare(chx, MICROSEC_COMPARE_FORMAT);

  if (cur > 3000){
  channel_select_counter = 0;
  receiver_watchdog = 0;
  if (error == 8 && start == 2)error = 0;
  }
  else channel_select_counter++;

  if (channel_select_counter == 1)channel1 = cur;
  if (channel_select_counter == 2)channel2 = cur;
  if (channel_select_counter == 3)channel3 = cur;
  if (channel_select_counter == 4)channel4 = cur;
  if (channel_select_counter == 5)channel5 = cur;
  if (channel_select_counter == 6)channel6 = cur;
}



void timer_setup(){

    // Input from RX (TIM1)
    InTim->setMode(chx, TIMER_INPUT_CAPTURE_RISING, ch1);

    InTim->setPrescaleFactor(84);
    InTim->setOverflow(0x10000, MICROSEC_FORMAT); 
    InTim->setCaptureCompare(1, 1000, MICROSEC_COMPARE_FORMAT);
    InTim->setCaptureCompare(2, 1000, MICROSEC_COMPARE_FORMAT);
    InTim->setCaptureCompare(3, 1000, MICROSEC_COMPARE_FORMAT);
    InTim->setCaptureCompare(4, 1000, MICROSEC_COMPARE_FORMAT);

    InTim->attachInterrupt(1, handler);


    // Output to ESCs (TIM4)
    OutTim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, ch1_out);
    OutTim->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, ch2_out);
    OutTim->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, ch3_out);
    OutTim->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, ch4_out);

    OutTim->setPrescaleFactor(84);
    OutTim->setOverflow(5001, MICROSEC_FORMAT);
    OutTim->setCaptureCompare(1, 1000, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(2, 1000, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(3, 1000, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(4, 1000, MICROSEC_COMPARE_FORMAT);


    // Resume timers
    InTim->resume();
    OutTim->resume();
}


void set_pwm(){
    OutTim->setCount(5000, MICROSEC_FORMAT);
    OutTim->setCaptureCompare(1, channel1, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(2, channel2, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(3, channel3, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(4, channel4, MICROSEC_COMPARE_FORMAT);
}