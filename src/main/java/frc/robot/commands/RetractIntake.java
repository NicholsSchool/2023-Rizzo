// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uprighter;

public class RetractIntake extends CommandBase {

  private Intake intake;
  private Uprighter uprighter;
  private Gripper gripper;

  public RetractIntake(Intake intakeSubsystem, Uprighter uprighterSubsystem, Gripper gripperSubsystem) {
    intake = intakeSubsystem;
    uprighter = uprighterSubsystem;
    gripper = gripperSubsystem; 
    addRequirements(intake, uprighter, gripper);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.spinIn();
    uprighter.spinIn();
    gripper.spinIn();
  }

  @Override
  public void end(boolean interrupted) {
    intake.lifterUp();
    intake.stop();
    uprighter.stop();
    gripper.stopSpinner();
    intake.flapperClose();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
