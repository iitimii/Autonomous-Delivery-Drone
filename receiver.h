#include <Arduino.h>

TIM_TypeDef *Instance_in = TIM4;
TIM_TypeDef *Instance_out = TIM1;

HardwareTimer *InTim = new HardwareTimer(Instance_in);
HardwareTimer *OutTim = new HardwareTimer(Instance_out);

#define ch1 PB6
#define ch2 PB7
#define ch3 PB8
#define ch4 PB9
#define ch1_out PA8
#define ch2_out PA9
#define ch3_out PA10
#define ch4_out PA11



uint32_t channel1, channel2, channel3, channel4;
int32_t cur1=0, cur2=0, cur3=0, cur4=0;
int32_t prev1, prev2, prev3, prev4;
int32_t ch1_val, ch2_val, ch3_val, ch4_val;
uint32_t loop_timer;

//all interupts call the same function, if coming from channel 5, capture from channel five

void handler_ch1(void){
    cur1 = InTim->getCaptureCompare(channel1, MICROSEC_COMPARE_FORMAT) - prev1;
    if (cur1 < 0) cur1 += 0x10000;
    if (cur1 <= 2000) ch1_val = cur1;
    prev1 = InTim->getCaptureCompare(channel1, MICROSEC_COMPARE_FORMAT);
}

void handler_ch2(void){
    cur2 = InTim->getCaptureCompare(channel2, MICROSEC_COMPARE_FORMAT) - prev2;
    if (cur2 < 0) cur2 += 0x10000; 
    if (cur2 <= 2000) ch2_val = cur2;
    prev2 = InTim->getCaptureCompare(channel2, MICROSEC_COMPARE_FORMAT);
}

void handler_ch3(void){
    cur3 = InTim->getCaptureCompare(channel3, MICROSEC_COMPARE_FORMAT) - prev3;
    if (cur3 < 0) cur3 += 0x10000;
    if (cur3 <= 2000) ch3_val = cur3;
    prev3 = InTim->getCaptureCompare(channel3, MICROSEC_COMPARE_FORMAT);
}

void handler_ch4(void){
    cur4 = InTim->getCaptureCompare(channel4, MICROSEC_COMPARE_FORMAT) - prev4;
    if (cur4 < 0) cur4 += 0x10000;
    if (cur4 <= 2000) ch4_val = cur4;
    prev4 = InTim->getCaptureCompare(channel4, MICROSEC_COMPARE_FORMAT);
}



void setup(){
    Serial.begin(57600);

    channel1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(ch1), PinMap_PWM));
    channel2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(ch2), PinMap_PWM));
    channel3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(ch3), PinMap_PWM));
    channel4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(ch4), PinMap_PWM));

    // Input from RX (TIM4)
    InTim->setMode(channel1, TIMER_INPUT_CAPTURE_BOTHEDGE, ch1);
    InTim->setMode(channel2, TIMER_INPUT_CAPTURE_BOTHEDGE, ch2);
    InTim->setMode(channel3, TIMER_INPUT_CAPTURE_BOTHEDGE, ch3);
    InTim->setMode(channel4, TIMER_INPUT_CAPTURE_BOTHEDGE, ch4);

    InTim->setPrescaleFactor(84);
    InTim->setOverflow(0x10000, MICROSEC_FORMAT); 
    InTim->setCaptureCompare(channel1, 1000, MICROSEC_COMPARE_FORMAT);
    InTim->setCaptureCompare(channel2, 1000, MICROSEC_COMPARE_FORMAT);
    InTim->setCaptureCompare(channel3, 1000, MICROSEC_COMPARE_FORMAT);
    InTim->setCaptureCompare(channel4, 1000, MICROSEC_COMPARE_FORMAT);

    InTim->attachInterrupt(channel1, handler_ch1);
    InTim->attachInterrupt(channel2, handler_ch2);
    InTim->attachInterrupt(channel3, handler_ch3);
    InTim->attachInterrupt(channel4, handler_ch4);


    // Output to ESCs (TIM1)
    OutTim->setMode(channel1, TIMER_OUTPUT_COMPARE_PWM1, ch1_out);
    OutTim->setMode(channel2, TIMER_OUTPUT_COMPARE_PWM1, ch2_out);
    OutTim->setMode(channel3, TIMER_OUTPUT_COMPARE_PWM1, ch3_out);
    OutTim->setMode(channel4, TIMER_OUTPUT_COMPARE_PWM1, ch4_out);

    OutTim->setPrescaleFactor(84);
    OutTim->setOverflow(5001, MICROSEC_FORMAT);
    OutTim->setCaptureCompare(channel1, 1000, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(channel2, 1000, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(channel3, 1000, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(channel4, 1000, MICROSEC_COMPARE_FORMAT);


    // Resume timers
    InTim->resume();
    OutTim->resume();
    Serial.println("Timers Resumed");
}


void loop(){
    loop_timer = micros() + 4000;
    OutTim->setCount(5000, MICROSEC_FORMAT);
    OutTim->setCaptureCompare(channel1, ch1_val, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(channel2, ch2_val, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(channel3, ch3_val, MICROSEC_COMPARE_FORMAT);
    OutTim->setCaptureCompare(channel4, ch4_val, MICROSEC_COMPARE_FORMAT);


    while (loop_timer > micros());
}
