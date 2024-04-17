// #include <Arduino.h>
// class Motors
// {
// private:
//     int motor_fr;
//     int motor_fl;
//     int motor_br;
//     int motor_bl;

// public:
//     Motors()
//     {
//         TIM_TypeDef *Instance_out = TIM4;
//         HardwareTimer *OutTim = new HardwareTimer(Instance_out);
//         motor_fr = 1000;
//         motor_fl = 1000;
//         motor_br = 1000;
//         motor_bl = 1000;
//     }

//     void set_esc_pwm()
//     {
//         OutTim->setCaptureCompare(1, motor_fr, MICROSEC_COMPARE_FORMAT); // PB6 FR
//         OutTim->setCaptureCompare(2, motor_fl, MICROSEC_COMPARE_FORMAT); // PB7 FL
//         OutTim->setCaptureCompare(3, motor_br, MICROSEC_COMPARE_FORMAT); // PB8 BR
//         OutTim->setCaptureCompare(4, motor_bl, MICROSEC_COMPARE_FORMAT); // PB9 BL
//         OutTim->setCount(5000, MICROSEC_FORMAT);
//     }

//     void idle();
//     void arm();
//     void disarm();
// };