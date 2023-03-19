// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class Nudge extends CommandBase {

  SwerveDrive swerveDrive;
  char dir;
  boolean fieldRelative;

  public Nudge(SwerveDrive swerveDrive, char dir, boolean fieldRelative) {

    this.swerveDrive = swerveDrive;
    this.dir = dir;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    switch (dir) {
      case 'r':
        swerveDrive.drive(-0.5, 0.0, 0, fieldRelative);
        break;
      case 'l':
        swerveDrive.drive(0.5, 0.0, 0, fieldRelative);
        break;
      case 'u':
        swerveDrive.drive(0.0, 0.5, 0, fieldRelative);
        break;
      case 'd':
        swerveDrive.drive(0.0, -0.5, 0, fieldRelative);
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
