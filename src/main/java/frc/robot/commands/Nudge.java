package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Nudges the robot in a specified direction.
 */
public class Nudge extends CommandBase {

  SwerveDrive swerveDrive;
  String direction;
  boolean fieldRelative;

  public Nudge(SwerveDrive _swerveDrive, String _direction, boolean _fieldRelative) {

    swerveDrive = _swerveDrive;
    direction = _direction;
    fieldRelative = _fieldRelative;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    switch (direction) {

      // DRIVER CONTROLS
      case "DRIVER NUDGE FORWARD":
        swerveDrive.drive(0.5, 0.0, 0, fieldRelative);
        break;
      case "DRIVER NUDGE BACKWARD":
        swerveDrive.drive(-0.5, 0.0, 0, fieldRelative);
        break;
      case "DRIVER NUDGE LEFT":
        swerveDrive.drive(0.0, 0.5, 0, fieldRelative);
        break;
      case "DRIVER NUDGE RIGHT":
        swerveDrive.drive(0.0, -0.5, 0, fieldRelative);
        break;

      // OPERATOR CONTROLS
      case "OPERATOR NUDGE FORWARD":
        swerveDrive.drive(-0.5, 0.0, 0, fieldRelative);
        break;
      case "OPERATOR NUDGE BACKWARD":
        swerveDrive.drive(0.5, 0.0, 0, fieldRelative);
        break;
      case "OPERATOR NUDGE LEFT":
        swerveDrive.drive(0.0, -0.5, 0, fieldRelative);
        break;
      case "OPERATOR NUDGE RIGHT":
        swerveDrive.drive(0.0, 0.5, 0, fieldRelative);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0.0, 0.0, 0.0, fieldRelative);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
