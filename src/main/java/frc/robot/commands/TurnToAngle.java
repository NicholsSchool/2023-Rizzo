package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class TurnToAngle extends CommandBase {

  PIDController pid;
  SwerveDrive swerveDrive;
  Double xSpeed;
  Double ySpeed;
  Double desiredAngle;

  static final double kP = 0.55;
  static final double kI = 0.01;
  static final double kD = 0.00;
  static final double kF = 0.00;
  static final double positionTolerance = 2.0f;

  public TurnToAngle(SwerveDrive _swerveDrive, Double _ySpeed, Double _xSpeed, Double _angle) {

    swerveDrive = _swerveDrive;
    xSpeed = _ySpeed;
    ySpeed = _xSpeed;
    desiredAngle = _angle;

    addRequirements(swerveDrive);

    pid = new PIDController(kP, kI, kD, kF);
    pid.enableContinuousInput(-180.0, 180.0);
    pid.setIntegratorRange(-2.0, 2.0);
    pid.setTolerance(positionTolerance);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double currentYaw = swerveDrive.getYaw();
    double angularRotation = pid.calculate(currentYaw, desiredAngle);
    swerveDrive.drive(xSpeed, ySpeed, angularRotation, true);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
