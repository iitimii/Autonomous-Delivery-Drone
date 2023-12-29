void handler(void) {
  measured_time = InTim->getCaptureCompare(chx, MICROSEC_COMPARE_FORMAT) - measured_time_start+15;
  if (measured_time < 0) measured_time += 0x10000;
  measured_time_start = InTim->getCaptureCompare(chx, MICROSEC_COMPARE_FORMAT);

  if (measured_time > 3000){
  channel_select_counter = 0;
  receiver_watchdog = 0;
  if (error == 8 && start == 2)error = 0;
  }
  else channel_select_counter++;
  // if (measured_time<1000)measured_time=1000;
  // if (measured_time>2000)measured_time=2000;

  if (channel_select_counter == 1)channel_1 = measured_time;
  if (channel_select_counter == 2)channel_2 = measured_time;
  if (channel_select_counter == 3)channel_3 = measured_time;
  if (channel_select_counter == 4)channel_4 = measured_time;
  if (channel_select_counter == 5)channel_5 = measured_time;
  if (channel_select_counter == 6)channel_6 = measured_time;
  if (channel_select_counter == 7)channel_7 = measured_time;
  if (channel_select_counter == 8)channel_8 = measured_time;
  if (channel_select_counter == 9)channel_9 = measured_time;
  if (channel_select_counter == 10)channel_10 = measured_time;
}



void timer_setup(){

    // Input from RX (TIM1)
    InTim->setMode(chx, TIMER_INPUT_CAPTURE_RISING, inTimPin);
    InTim->setPrescaleFactor(84);
    InTim->setOverflow(0x10000, MICROSEC_FORMAT); 
    InTim->setCaptureCompare(1, 1000, MICROSEC_COMPARE_FORMAT);
    InTim->attachInterrupt(chx, handler);
    


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


void set_esc_pwm(){
    OutTim->setCaptureCompare(1, esc_1, MICROSEC_COMPARE_FORMAT); //PB6 FR
    OutTim->setCaptureCompare(2, esc_4, MICROSEC_COMPARE_FORMAT); //PB7 FL
    OutTim->setCaptureCompare(3, esc_3, MICROSEC_COMPARE_FORMAT); //PB8 BL
    OutTim->setCaptureCompare(4, esc_2, MICROSEC_COMPARE_FORMAT); //PB9 BR
    OutTim->setCount(5000, MICROSEC_FORMAT);
}