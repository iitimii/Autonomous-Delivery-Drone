// #include <string.h>

// HardwareSerial rxSerial(PA3, PA2);

// #define IBUS_BUFFSIZE 32    // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
// #define IBUS_MAXCHANNELS 10 // My TX only has 10 channels, no point in polling the rest

// static uint8_t ibusIndex = 0;
// static uint8_t ibus[IBUS_BUFFSIZE] = {0};
// static uint16_t rcValue[IBUS_MAXCHANNELS];

// static bool rxFrameDone;

// void rxsetup() 
// {
//     // Output to ESCs (TIM4)
//     OutTim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, ch1_out);
//     OutTim->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, ch2_out);
//     OutTim->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, ch3_out);
//     OutTim->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, ch4_out);
//     OutTim->setPrescaleFactor(84);
//     OutTim->setOverflow(5001, MICROSEC_FORMAT);
//     OutTim->setCaptureCompare(1, 1000, MICROSEC_COMPARE_FORMAT);
//     OutTim->setCaptureCompare(2, 1000, MICROSEC_COMPARE_FORMAT);
//     OutTim->setCaptureCompare(3, 1000, MICROSEC_COMPARE_FORMAT);
//     OutTim->setCaptureCompare(4, 1000, MICROSEC_COMPARE_FORMAT);


//     // Resume timers
//     OutTim->resume();

//   rxSerial.begin(115200); 
//   rxSerial.println("setup done!");
// }


// void readRx()
// {
    
//   rxFrameDone = false;
  
//   if (rxSerial.available())
//   {
//     uint8_t val = rxSerial.read();
//     // Look for 0x2040 as start of packet
//     if (ibusIndex == 0 && val != 0x20)
//     {
//       ibusIndex = 0;
//       return;
//     }
//     if (ibusIndex == 1 && val != 0x40) 
//     {
//       ibusIndex = 0;
//       return;
//     }

//     if (ibusIndex == IBUS_BUFFSIZE)
//     {
//       ibusIndex = 0;
//       int high=3;
//       int low=2;
//       for(int i=0;i<IBUS_MAXCHANNELS; i++)
//       {
//         //left shift away the first 8 bits of the first byte and add the whole value of the previous one
//         rcValue[i] = (ibus[high] << 8) + ibus[low];
//         rxSerial.print(rcValue[i]);
//         rxSerial.print("     ");
//         high += 2;
//         low += 2;
//       }
//       rxSerial.println();
//       rxFrameDone = true;
//       return;
//     }
//     else
//     {
//       ibus[ibusIndex] = val;
//       ibusIndex++;
//     }
//   }


//     channel_1 = rcValue[0];
//     channel_2 = rcValue[1];
//     channel_3 = rcValue[2];
//     channel_4 = rcValue[3];
//     channel_5 = rcValue[4];
//     channel_6 = rcValue[5];
//     channel_7 = rcValue[6];
//     channel_8 = rcValue[7];
//     channel_9 = rcValue[8];
//     channel_10 = rcValue[9];
// }

// void readRx2(){
//     rxFrameDone = false;
//     if (rxSerial.available()){
//         uint8_t val = rxSerial.read();
//         if (ibusIndex == 0 && val == 0x20)
//             {
//             ibusIndex = 1;
//             uint8_t val = rxSerial.read();
//             if (ibusIndex == 1 && val == 0x40) 
//                 {
//                 ibusIndex = 2;
//                 for (int i = 2; i < IBUS_BUFFSIZE; i++)
//                     {
//                     ibus[i] = rxSerial.read();
//                     }

//                     ibusIndex = 0;
//                     int high=3;
//                     int low=2;
//                     for(int i=0;i<IBUS_MAXCHANNELS; i++)
//                     {
//                         //left shift away the first 8 bits of the first byte and add the whole value of the previous one
//                         rcValue[i] = (ibus[high] << 8) + ibus[low];
//                         // rxSerial.print(rcValue[i]);
//                         // rxSerial.print("     ");
//                         high += 2;
//                         low += 2;
//                     }
//                     // rxSerial.println();
//                     rxFrameDone = true;
//                     channel_1 = rcValue[0];
//                     channel_2 = rcValue[1];
//                     channel_3 = rcValue[2];
//                     channel_4 = rcValue[3];
//                     channel_5 = rcValue[4];
//                     channel_6 = rcValue[5];
//                     channel_7 = rcValue[6];
//                     channel_8 = rcValue[7];
//                     channel_9 = rcValue[8];
//                     channel_10 = rcValue[9];

//                 }

//                 else {
//                 ibusIndex = 0;
//                 return;}

//             }
//             else {
//                 ibusIndex = 0;
//                 return;}
//     }

//     else ibusIndex = 0;
// }


