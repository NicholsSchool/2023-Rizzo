// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uprighter;

public class ShootCube extends CommandBase {
    Intake intake;
    Uprighter uprighter;
    Gripper gripper;


  public ShootCube( Gripper gripper, Intake intake, Uprighter uprighter ) {

    this.gripper = gripper;
    this.intake = intake; 
    this.uprighter = uprighter; 

    addRequirements( this.gripper, this.intake, this.uprighter );
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  @Override
  public void execute()
  {
    gripper.spinOut();
    uprighter.spinOut();
    intake.spinOut();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    gripper.stop();
    uprighter.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
