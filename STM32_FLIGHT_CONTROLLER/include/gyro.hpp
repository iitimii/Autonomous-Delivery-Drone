uint8_t nbytes = 14;
void gyro_setup()
{
  delay(50);
  HWire.beginTransmission(gyro_address); // Start communication with the MPU-6050.
  HWire.write(0x6B);                     // We want to write to the PWR_MGMT_1 register (6B hex).
  HWire.write(0x00);                     // Set the register bits as 00000000 to activate the gyro.
  HWire.endTransmission();               // End the transmission with the gyro.

  HWire.beginTransmission(gyro_address); // Start communication with the MPU-6050.
  HWire.write(0x1B);                     // We want to write to the GYRO_CONFIG register (1B hex).
  HWire.write(0x08);                     // Set the register bits as 00001000 (500dps full scale).
  HWire.endTransmission();               // End the transmission with the gyro.

  HWire.beginTransmission(gyro_address); // Start communication with the MPU-6050.
  HWire.write(0x1C);                     // We want to write to the ACCEL_CONFIG register (1A hex).
  HWire.write(0x10);                     // Set the register bits as 00010000 (+/- 8g full scale range).
  HWire.endTransmission();               // End the transmission with the gyro.

  HWire.beginTransmission(gyro_address); // Start communication with the MPU-6050.
  HWire.write(0x1A);                     // We want to write to the CONFIG register (1A hex).
  HWire.write(0x03);                     // Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  HWire.endTransmission();               // End the transmission with the gyro.
  delay(50);
}

void read_gyro()
{
  HWire.beginTransmission(gyro_address);             // Start communication with the gyro.
  HWire.write(0x3B);                                 // Start reading @ register 43h and auto increment with every read.
  HWire.endTransmission();                           // End the transmission.
  HWire.requestFrom(gyro_address, nbytes);           // Request 14 bytes from the MPU 6050.
  acc_y = HWire.read() << 8 | HWire.read();          // Add the low and high byte to the acc_x variable.
  acc_x = HWire.read() << 8 | HWire.read();          // Add the low and high byte to the acc_y variable.
  acc_z = HWire.read() << 8 | HWire.read();          // Add the low and high byte to the acc_z variable.
  temperature = HWire.read() << 8 | HWire.read();    // Add the low and high byte to the temperature variable.
  roll_velocity = HWire.read() << 8 | HWire.read();  // Read high and low part of the angular data.
  pitch_velocity = HWire.read() << 8 | HWire.read(); // Read high and low part of the angular data.
  yaw_velocity = HWire.read() << 8 | HWire.read();   // Read high and low part of the angular data.

  acc_z *= -1;
  acc_x *= -1;
  // acc_y *= -1;

  // acc_y -= acc_y_cal;                         //Subtact the manual accelerometer pitch calibration value.
  // acc_x -= acc_x_cal;                          //Subtact the manual accelerometer roll calibration value.
  roll_velocity -= roll_vel_cal;   // Subtact the manual gyro roll calibration value.
  pitch_velocity -= pitch_vel_cal; // Subtact the manual gyro pitch calibration value.
  yaw_velocity -= yaw_vel_cal;     // Subtact the manual gyro yaw calibration value.
}

void calibrate_gyro()
{
  cal_int = 0; // Set the cal_int variable to zero.
  if (cal_int != 2000)
  {
    // Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000; cal_int++)
    { // Take 2000 readings for calibration.
      if (cal_int % 25 == 0)
        digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED)); // Change the led status every 125 readings to indicate calibration.
      read_gyro();                                                    // Read the gyro output.
      roll_vel_cal += roll_velocity;                                  // Ad roll value to roll_vel_cal.
      pitch_vel_cal += pitch_velocity;                                // Ad pitch value to pitch_vel_cal.
      yaw_vel_cal += yaw_velocity;                                    // Ad yaw value to yaw_vel_cal.
      // acc_x_cal += acc_x;
      // acc_y_cal += acc_y;
      delay(4); // Small delay to simulate a 250Hz loop during calibration.
    }

    // Now that we have 2000 measures, we need to divide by 2000 to get the average gyro offset.
    roll_vel_cal /= 2000;  // Divide the roll total by 2000.
    pitch_vel_cal /= 2000; // Divide the pitch total by 2000.
    yaw_vel_cal /= 2000;   // Divide the yaw total by 2000.
    // acc_x_cal /= 2000;
    // acc_y_cal /= 2000;
  }
}
