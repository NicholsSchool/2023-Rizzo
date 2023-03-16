package frc.robot.utils;

import com.revrobotics.SparkMaxPIDController;

// REV Robotics PID Gains Utility Class
public class RevPIDGains {
  public final double p;
  public final double i;
  public final double d;

  public RevPIDGains(double _p, double _i, double _d) {
    p = _p;
    i = _i;
    d = _d;
  }

  public static void setSparkMaxGains(SparkMaxPIDController _controller, RevPIDGains _gains) {
    _controller.setP(_gains.p);
    _controller.setI(_gains.i);
    _controller.setD(_gains.d);
  }
}
