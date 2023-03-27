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

    double currentYaw = swerveDrive.getYaw();
    double difference = desiredAngle - currentYaw;
    double error = difference;
    double angularRotation = 0.0;

    if (difference > 180) {
      difference -= 360;
    } else if (difference < -180) {
      difference += 360;
    }

    if (difference > 0) {
      // turn clockwise
    } else if (difference < 0) {
      // turn counter-clockwise

    }

    swerveDrive.drive(xSpeed, ySpeed, angularRotation, true);

  }

  public void executed() {

    double currentYaw = swerveDrive.getYaw();
    double error = desiredAngle - currentYaw;
    double angularRotation = 0.0;
    double Kp = 0.5;

    if (Math.abs(error) > 1) {
      // Calculate the output
      double output = Kp * error;
      // Limit output to -1.0 to 1.0
      output = Math.max(-1.0, Math.min(1.0, output));
      if (output > 0) {
        // Rotate clockwise
        angularRotation = Math.PI * -output;
      } else if (output < 0) {
        // Rotate counter-clockwise
        angularRotation = Math.PI * output;
      }
      swerveDrive.drive(xSpeed, ySpeed, angularRotation, true);
    } else {
      System.out.println("Reached desired angle: " + desiredAngle);
    }

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
