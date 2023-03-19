// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CBArm;
import frc.robot.utils.PID;
import frc.robot.Constants.ArmConstants;

public class GoToPos extends CommandBase {

  private double pos;
  private CBArm arm;

  private PID pid;

  public GoToPos(double pos, CBArm arm) {
    this.pos = pos;
    this.arm = arm;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PID(ArmConstants.P, ArmConstants.I, ArmConstants.D, ArmConstants.dt, pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.update(arm.getPosition());

    System.out.println("current pos : " + arm.getPosition() + ", target pos : " + pos);

    arm.move(pid.getOutput());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
