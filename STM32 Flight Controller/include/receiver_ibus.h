// #include <Arduino.h>
// // #include <IBusBM.h>


// HardwareSerial rxSerial(PA12, PA11);
// TIM_TypeDef *Instance_out = TIM4;
// HardwareTimer *OutTim = new HardwareTimer(Instance_out);



// // static bool rxFrameDone;


// uint32_t channel1, channel2, channel3, channel4, channel5, channel6;
// int32_t cur, prev;



// void timer_setup(){
//     rxSerial.begin(115200);
//     // Output to ESCs (TIM4)
//     // OutTim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, ch1_out);
//     // OutTim->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, ch2_out);
//     // OutTim->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, ch3_out);
//     // OutTim->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, ch4_out);

//     // OutTim->setPrescaleFactor(84);
//     // OutTim->setOverflow(5001, MICROSEC_FORMAT);
//     // OutTim->setCaptureCompare(1, 1000, MICROSEC_COMPARE_FORMAT);
//     // OutTim->setCaptureCompare(2, 1000, MICROSEC_COMPARE_FORMAT);
//     // OutTim->setCaptureCompare(3, 1000, MICROSEC_COMPARE_FORMAT);
//     // OutTim->setCaptureCompare(4, 1000, MICROSEC_COMPARE_FORMAT);


//     // Resume timers
//     // OutTim->resume();
// }


// void read_ibus(){
//     // rxFrameDone = false;
//     if(rxSerial.available()){
//         uint8_t val = rxSerial.read();

//         if (ibus_index==0 && val!=0x20){
//             ibus_index = 0;
//             return;
//         }
//         if (ibus_index==1 && val!=0x40){
//             ibus_index = 0;
//             return;
//         }
//         if (ibus_index==IBUS_BUFFSIZE){
//             ibus_index = 0;
//             int high = 3;
//             int low =2;

//             for(int i=0; i<IBUS_MAXCHANNELS; i++){
//                 rc_value[i] = (ibus[high]<<8)+ibus[low];
//                 high += 2;
//                 low += 2;
//             }
//             rxSerial.println();
//             // rxFrameDone = true;
//             return;
//         }
//         else{
//             ibus[ibus_index] = val;
//             ibus_index++;
//         }
//     }

// }

// void read_ibus2(){
//     if(rxSerial.available()){
//         uint8_t val = rxSerial.read();
//         if (ibus_index==0 && val==0x20){
//             ibus_index++;
//             val = rxSerial.read();
//             if(val==0x40){
//                 for(ibus_index=2;ibus_index<IBUS_BUFFSIZE;ibus_index++){
//                     ibus[ibus_index] = val;
//                 }
//                 int high = 3;
//                 int low = 2;
//                 for(int i=0; i<IBUS_MAXCHANNELS; i++){
//                 rc_value[i] = (ibus[high]<<8)+ibus[low];
//                 high += 2;
//                 low += 2;
//             }
//                 rxSerial.println();
//                 // rxFrameDone = true;
//                 return;
//             }
//         }
//         else{
//             ibus_index=0;
//         }
//     }
// }



// void set_pwm(){
//     // read_ibus();
//     // if (rxFrameDone);
//     OutTim->setCount(5000, MICROSEC_FORMAT);
//     OutTim->setCaptureCompare(1, channel1, MICROSEC_COMPARE_FORMAT);
//     OutTim->setCaptureCompare(2, channel2, MICROSEC_COMPARE_FORMAT);
//     OutTim->setCaptureCompare(3, channel3, MICROSEC_COMPARE_FORMAT);
//     OutTim->setCaptureCompare(4, channel4, MICROSEC_COMPARE_FORMAT);
// }