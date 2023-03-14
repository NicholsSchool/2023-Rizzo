package frc.robot.utils;

import static frc.robot.Constants.MILLISECS_PER_SEC;

public class PID {
  private double goal;
  private double output;

  private final double kP;
  private final double kI;
  private final double kD;
  private final double dt;

  private double p;
  private double i;
  private double d;

  private double time;
  private double lTime;

  private double error;
  private double lError;

  public PID(double kP, double kI, double kD, double dt, double goal) {
    super();

    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.dt = MILLISECS_PER_SEC * dt;
    this.goal = goal;

    this.time = System.currentTimeMillis();
    this.lTime = this.time;
  }

  public void update(double current) {
    error = goal - current;

    time = System.currentTimeMillis();
    double deltaTime = time - lTime;

    if (deltaTime >= dt) {
      p = error;
      i += error * deltaTime;
      d = (error - lError) / deltaTime;

      lTime = time;
      lError = error;
      output = (kP * p) + (kI * i) + (kD * d);
    }
  }

  public void update(double current, double newGoal) {
    goal = newGoal;
    error = goal - current;

    time = System.currentTimeMillis();
    double deltaTime = time - lTime;

    if (deltaTime >= dt) {
      p = error;
      i += error * deltaTime;
      d = (error - lError) / deltaTime;

      lTime = time;
      lError = error;
      output = (kP * p) + (kI * i) + (kD * d);
    }
  }

  public double getOutput() {
    return output;
  }
}
