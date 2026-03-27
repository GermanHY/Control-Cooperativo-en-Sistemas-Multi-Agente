// =========================
// PID velocidad
// =========================
double ControladorVelocidad(double input_vel, double setpoint_vel,
                           double &error_ant, double &integral,
                           double Kp, double Ki, double Kd,
                           double sat_pwm) {
  const double Ts = 0.02;  // período fijo de 20 ms garantizado por vTaskDelayUntil

  // Error actual
  double error = setpoint_vel - input_vel;

  // Término proporcional
  double termino_P = Kp * error;

  // Término integral con anti-windup
  integral += error * Ts;
  integral = constrain(integral, -sat_pwm, sat_pwm);
  double termino_I = Ki * integral;

  // Término derivativo (diferencia hacia atrás)
  double derivada = (error - error_ant) / Ts;
  double termino_D = Kd * derivada;

  // Salida total
  double salida = termino_P + termino_I + termino_D;

  // Saturación de salida
  salida = constrain(salida, -sat_pwm, sat_pwm);

  // Actualizar memoria del error anterior
  error_ant = error;

  return salida;
}

// =========================
// PID distancia
// =========================
double ControladorDistancia(double input_d, double setpoint_d,
                           double &error_ant, double &integral,
                           double Kp, double Ki, double Kd,
                           double sat_max) {
  return 0.0;
}

// =========================
// PID angulo
// =========================
float ControladorOrientacion(int error, int &prev, float Kp, float Ki, float Kd) {
  const float Ts = 0.02f;  // período fijo de 20 ms garantizado por vTaskDelayUntil

  // Término proporcional
  float termino_P = Kp * (float)error;

  // Término derivativo (diferencia hacia atrás)
  float derivada  = ((float)error - (float)prev) / Ts;
  float termino_D = Kd * derivada;

  // Salida total
  float salida = termino_P + termino_D;

  // Actualizar memoria (prev se pasa por referencia, el cambio persiste afuera)
  prev = error;

  return salida;
}

