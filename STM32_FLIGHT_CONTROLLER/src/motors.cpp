
#define ch1_out PB6
#define ch2_out PB7
#define ch3_out PB8
#define ch4_out PB9

int16_t motor_fr=1000, motor_fl=1000, motor_br=1000, motor_bl=1000;
int16_t manual_throttle;
int16_t throttle, takeoff_throttle, cal_int;

TIM_TypeDef *Instance_out = TIM4;
HardwareTimer *OutTim = new HardwareTimer(Instance_out);