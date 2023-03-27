package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Rotate robot chassis to a predefined position relative to the field.
 */
public class Rotate extends CommandBase {

  SwerveDrive swerveDrive;
  Double xSpeed;
  Double ySpeed;
  Double desiredAngle;

  public Rotate(SwerveDrive _swerveDrive, Double _ySpeed, Double _xSpeed, Double _angle) {

    swerveDrive = _swerveDrive;
    xSpeed = _ySpeed;
    ySpeed = _xSpeed;
    desiredAngle = _angle;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double currentYaw = 3;
    double desiredAngle = -113;
    double difference = desiredAngle - currentYaw;
    double error = 0.0;
    double angularRotation = 0.0;
    double kP = 0.85;

    if (Math.abs(difference) > 180) {
      error = difference - (360 * (Math.abs(difference) / difference));
    } else {
      error = difference;
    }

    angularRotation = error / 180 * (Math.PI * kP);

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
