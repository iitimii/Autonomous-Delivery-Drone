void blueled(int8_t level) 
{
  digitalWrite(STM32_board_LED, !level);          
}

void blink_led()
{
  for (count_var = 0; count_var < 1250; ++count_var)
  {
    if (count_var % 125 == 0)
    {
      digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));
    }
    delay(4);
  }
  count_var = 0;
}