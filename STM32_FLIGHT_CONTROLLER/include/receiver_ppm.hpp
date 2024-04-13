void handler(void) {
  measured_time = InTim->getCaptureCompare(chx, MICROSEC_COMPARE_FORMAT) - measured_time_start;
  if (measured_time < 0) measured_time += 0xFFFF;
  measured_time-=25;
  measured_time_start = InTim->getCaptureCompare(chx, MICROSEC_COMPARE_FORMAT);

  if (measured_time > 3000){
  channel_select_counter = 0;
  }
  else channel_select_counter++;

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
    delay(50);
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
    delay(50);
}


void set_esc_pwm(){
    OutTim->setCaptureCompare(1, motor_fr, MICROSEC_COMPARE_FORMAT); //PB6 FR
    OutTim->setCaptureCompare(2, motor_fl, MICROSEC_COMPARE_FORMAT); //PB7 FL
    OutTim->setCaptureCompare(3, motor_br, MICROSEC_COMPARE_FORMAT); //PB8 BR
    OutTim->setCaptureCompare(4, motor_bl, MICROSEC_COMPARE_FORMAT); //PB9 BL
    OutTim->setCount(5000, MICROSEC_FORMAT);
}

void wait_for_receiver()
{
  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 800 || channel_4 < 990)
  {
    error = 4;
    send_telemetry();
    delay(4);
  }
}