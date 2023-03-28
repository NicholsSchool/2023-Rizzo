package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

/**
 * Rotate robot chassis to a predefined position relative to the field.
 */
public class Rotate extends CommandBase {

  SwerveDrive swerveDrive;
  Double desiredAngle;

  public Rotate(SwerveDrive _swerveDrive, Double _angle) {

    swerveDrive = _swerveDrive;
    desiredAngle = _angle;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double xSpeed = -RobotContainer.driverOI.getLeftY();
    double ySpeed = RobotContainer.driverOI.getLeftX();

    double currentYaw = swerveDrive.getYaw();
    double difference = desiredAngle - currentYaw;
    double error, angularRotation = 0.0;
    double kP = 0.63;

    if (Math.abs(difference) > 180) {
      error = difference - (360 * (Math.abs(difference) / difference));
    } else {
      error = difference;
    }

    angularRotation = error / 180 * (Math.PI * kP);

    // System.out.println(ySpeed);

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
