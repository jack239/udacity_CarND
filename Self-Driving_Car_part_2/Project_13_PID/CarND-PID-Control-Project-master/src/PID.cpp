#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = i_error = d_error = 0;
  is_first = true;
}

void PID::UpdateError(double cte) {
    if (!is_first) {
        d_error = cte - p_error;
    } else {
        is_first = false;
        d_error = 0;
    }
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
  return -(Kp * p_error + Kd * d_error + Ki * i_error);
}