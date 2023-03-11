package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Gripper;

public class Gripper_Outtake extends ParallelCommandGroup {
  
  
  public Gripper_Outtake( Gripper gripper ) {
    addCommands( 
      new InstantCommand(() ->  gripper.openPincher()), 
      new InstantCommand(() ->  gripper.spinOut()));
  }
}
