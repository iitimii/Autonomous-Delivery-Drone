void calculate_pid(void){
  pid_roll_setpoint = 0;
  if (channel_1 > 1508)pid_roll_setpoint = channel_1 - 1508;
  else if (channel_1 < 1492)pid_roll_setpoint = channel_1 - 1492;
  pid_roll_setpoint = (pid_roll_setpoint*max_angle)/500;
  pid_roll_setpoint -= angle_roll;
  pid_error_temp = pid_roll_setpoint - gyro_roll_input*0.3;
  p_term_roll = pid_p_gain_roll * pid_error_temp;
  i_term_roll = i_term_roll + (pid_i_gain_roll * pid_error_temp);
  if(i_term_roll > pid_max_roll)i_term_roll = pid_max_roll;
  else if(i_term_roll < pid_max_roll * -1)i_term_roll = pid_max_roll * -1;
  d_term_roll = pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  pid_output_roll = p_term_roll + i_term_roll + d_term_roll;
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  pid_last_roll_d_error = pid_error_temp;

  pid_pitch_setpoint = 0;
  if (channel_2 > 1508)pid_pitch_setpoint = channel_2 - 1508;
  else if (channel_1 < 1492)pid_pitch_setpoint = channel_1 - 1492;
  pid_pitch_setpoint = (pid_pitch_setpoint*max_angle)/500;
  pid_pitch_setpoint -= angle_pitch;
  pid_error_temp = pid_pitch_setpoint - gyro_pitch_input*0.3;
  p_term_pitch = pid_p_gain_pitch * pid_error_temp;
  i_term_pitch = i_term_pitch + (pid_i_gain_pitch * pid_error_temp);
  if(i_term_pitch > pid_max_pitch)i_term_pitch = pid_max_pitch;
  else if(i_term_pitch < pid_max_pitch * -1)i_term_pitch = pid_max_pitch * -1;
  d_term_pitch = pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  pid_output_pitch = p_term_pitch + i_term_pitch + d_term_pitch;
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
  pid_last_pitch_d_error = pid_error_temp;

  pid_yaw_setpoint = 0;
  if (channel_4 > 1508)pid_yaw_setpoint = channel_4 - 1508;
  else if (channel_1 < 1492)pid_yaw_setpoint = channel_1 - 1492;
  pid_error_temp = pid_yaw_setpoint - gyro_yaw_input;
  p_term_yaw = pid_p_gain_yaw * pid_error_temp;
  i_term_yaw = i_term_yaw + (pid_i_gain_yaw * pid_error_temp);
  if(i_term_yaw > pid_max_yaw)i_term_yaw = pid_max_yaw;
  else if(i_term_yaw < pid_max_yaw * -1)i_term_yaw = pid_max_yaw * -1;
  d_term_yaw = pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  pid_output_yaw = p_term_yaw + i_term_yaw + d_term_yaw;
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
  pid_last_yaw_d_error = pid_error_temp;
}

 //JOOPS
//   pid_roll_setpoint -= angle_roll*15;
//   pid_roll_setpoint /= 3;
//   pid_error_temp = gyro_roll_input - pid_roll_setpoint;
// // OR
//   drone_roll_state = (angle_roll * 0.9) + (gyro_roll_input * 0.1);
//   pid_error_temp = pid_roll_setpoint - drone_roll_state;
  //OR


