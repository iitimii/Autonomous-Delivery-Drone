///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the various registers of the MPU-6050 are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t nbytes = 14;
void gyro_setup(void) {
  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
  HWire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  HWire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
  HWire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
  HWire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  HWire.endTransmission();                                      //End the transmission with the gyro.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_gyro(void) {
  HWire.beginTransmission(gyro_address);                       //Start communication with the gyro.
  HWire.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
  HWire.endTransmission();                                     //End the transmission.
  HWire.requestFrom(gyro_address, nbytes);                         //Request 14 bytes from the MPU 6050.
  acc_x = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_x variable.
  acc_y = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_y variable.
  acc_z = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_z variable.
  temperature = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the temperature variable.
  gyro_pitch = HWire.read() << 8 | HWire.read();                //Read high and low part of the angular data.
  gyro_roll = HWire.read() << 8 | HWire.read();               //Read high and low part of the angular data.
  gyro_yaw = HWire.read() << 8 | HWire.read();                 //Read high and low part of the angular data.
  gyro_pitch *= -1;                                             //Invert gyro_pitch due to the fact that the IMU is upside down.
  gyro_roll *= -1;                                              //Invert gyro_roll due to the fact that the IMU is upside down.
  gyro_yaw *= -1;
  acc_y *= -1;

  acc_y -= acc_y_cal;                         //Subtact the manual accelerometer pitch calibration value.
  acc_x -= acc_x_cal;                          //Subtact the manual accelerometer roll calibration value.
  gyro_roll -= gyro_roll_cal;                     //Subtact the manual gyro roll calibration value.
  gyro_pitch -= gyro_pitch_cal;                   //Subtact the manual gyro pitch calibration value.
  gyro_yaw -= gyro_yaw_cal;                       //Subtact the manual gyro yaw calibration value.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This subroutine handles the calibration of the gyro. It stores the avarage gyro offset of 2000 readings.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {
  cal_int = 0;                                                                        //Set the cal_int variable to zero.
  if (cal_int != 2000) {
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
      if (cal_int % 25 == 0) digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));                    //Change the led status every 125 readings to indicate calibration.
      read_gyro();                                                                //Read the gyro output.
      gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                                       //Ad yaw value to gyro_yaw_cal.
      acc_x_cal += acc_x;                                                      
      acc_y_cal += acc_y;
      delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
    }
    digitalWrite(RED_LED_PIN, HIGH);                                                                     //Set output PB3 low.
    
    //Now that we have 2000 measures, we need to divide by 2000 to get the average gyro offset.
    gyro_roll_cal /= 2000;                                                            //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                                           //Divide the pitch total by 2000.
    gyro_yaw_cal /= 2000;                                                             //Divide the yaw total by 2000.
    acc_x_cal /= 2000;
    acc_y_cal /= 2000;
  }
}


