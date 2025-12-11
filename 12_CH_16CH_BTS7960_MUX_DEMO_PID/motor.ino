void motor(int LPWM, int RPWM) {

  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  // ----- LEFT MOTOR -----
  if (LPWM > 0) {
    // Forward
    analogWrite(left_motor_forward, LPWM);
    analogWrite(left_motor_backward, 0);
  } else if (LPWM < 0) {
    // Backward
    analogWrite(left_motor_forward, 0);
    analogWrite(left_motor_backward, -LPWM);
  }

  // ----- RIGHT MOTOR -----
  if (RPWM > 0) {
    // Forward
    analogWrite(right_motor_forward, RPWM);
    analogWrite(right_motor_backward, 0);
  } else if (RPWM < 0) {
    // Backward
    analogWrite(right_motor_forward, 0);
    analogWrite(right_motor_backward, -RPWM);
  }
}