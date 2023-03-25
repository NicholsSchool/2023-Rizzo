// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class Nudge extends CommandBase {

  SwerveDrive swerveDrive;
  String direction;
  boolean fieldRelative;

  public Nudge(SwerveDrive _swerveDrive, String _direction, boolean _fieldRelative) {

    this.swerveDrive = _swerveDrive;
    this.direction = _direction;
    this.fieldRelative = _fieldRelative;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    switch (direction) {

      // DRIVER CONTROLS
      case "DRIVER FORWARD":
        swerveDrive.drive(0.5, 0.0, 0, fieldRelative);
        break;
      case "DRIVER BACKWARD":
        swerveDrive.drive(-0.5, 0.0, 0, fieldRelative);
        break;
      case "DRIVER LEFT":
        swerveDrive.drive(0.0, 0.5, 0, fieldRelative);
        break;
      case "DRIVER RIGHT":
        swerveDrive.drive(0.0, -0.5, 0, fieldRelative);
        break;

      // OPERATOR CONTROLS
      case "OPERATOR FORWARD":
        swerveDrive.drive(-0.5, 0.0, 0, fieldRelative);
        break;
      case "OPERATOR BACKWARD":
        swerveDrive.drive(0.5, 0.0, 0, fieldRelative);
        break;
      case "OPERATOR LEFT":
        swerveDrive.drive(0.0, -0.5, 0, fieldRelative);
        break;
      case "OPERATOR RIGHT":
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
