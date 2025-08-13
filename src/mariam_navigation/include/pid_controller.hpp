// Simple PID controller
class PIDController {
  public:
  PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), 
    prev_error_(0.0), integral_(0.0) {}
    
  double compute(double error, double dt) {
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    prev_error_ = error;
    return output;
  }
  
  void reset() {
    prev_error_ = 0.0;
    integral_ = 0.0;
  }

  private:
  double kp_, ki_, kd_;
  double prev_error_;
  double integral_;
};
