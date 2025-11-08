void calculate_pressure() {
  if (counter == 0) {  //20 turda bi okuma kodu
    MS5611.read();
    smooth.add(MS5611.getPressure());
    counter = 10;
  }
  counter--;

  actual_pressure = smooth.get();
  quadprops.baro_height = actual_pressure;
  KalmanPosVel();
  actual_pressure_2 = quadprops.kalmanvel_z;

  if (switch2 == 0 && thrust > 1400 && thrust < 1450) {
    if (manual_altitude_change == 1) {
      pressure_parachute_previous = actual_pressure * 10;
    }
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30)
      parachute_rotating_mem_location = 0;

    if (switch2 == 0 && thrust < 1450 && thrust > 1400) {  //If the quadcopter is in altitude mode and flying.
      if (hold == 0) {
        pid_altitude_setpoint = actual_pressure;
        hold = 1;
      }  //If not yet set, set the PID altitude setpoint.
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      manual_altitude_change = 0;  //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;         //Set the manual_throttle variable to 0.
      if (thrust > 1450) {         //If the throtttle is increased above 1600us (60%).
        manual_altitude_change = 1;                                                    //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                       //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = static_cast<int16_t>((thrust - 1450) / 3);                   //To prevent very fast changes in hight limit the function of the throttle.
      }
      if (thrust < 1400) {                                                             //If the throttlle is lowered below 1400us (40%).
        manual_altitude_change = 1;                                                    //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                       //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = static_cast<int16_t>((thrust - 1400) / 5);                   //To prevent very fast changes in hight limit the function of the throttle.
      }

      //Calculate the PID output of the altitude hold.
      pid_altitude_input = actual_pressure;                        //Set the setpoint (pid_altitude_input) of the PID-controller.
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;  //Calculate the error between the setpoint and the actual pressure value.

      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
      pid_error_gain_altitude = 0;  //Set the pid_error_gain_altitude to 0.
      if (pid_error_temp > 10 || pid_error_temp < -10) {  //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        pid_error_gain_altitude = (fabs(pid_error_temp) - 10) / 20.0;  //The positive pid_error_gain_altitude variable is calculated based based on the error.
        if (pid_error_gain_altitude > 3)
          pid_error_gain_altitude = 3;  //To prevent extreme P-gains it must be limited to 3.
      }

      //In the following section the I-output is calculated. It's an accumulation of errors over time.
      //The time factor is removed as the program loop runs at 250Hz.
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude)
        pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)
        pid_i_mem_altitude = pid_max_altitude * -1;
      pid_output_altitude = (100 * (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle);
      if (pid_output_altitude > pid_max_altitude)
        pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)
        pid_output_altitude = pid_max_altitude * -1;
    }

  } else {
    hold = 0;
  }
}
