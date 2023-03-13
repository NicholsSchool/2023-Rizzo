package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uprighter;

public class DeployIntake extends CommandBase {

  private Intake intake;
  private Uprighter uprighter;
  private Gripper gripper; 

  public DeployIntake(Intake intakeSubsystem, Uprighter uprighterSubsystem, Gripper gripperSubsystem) {
    intake = intakeSubsystem;
    uprighter = uprighterSubsystem;
    gripper = gripperSubsystem; 
    addRequirements(intake, uprighter, gripper);
  }

  @Override
  public void initialize() {
    intake.lifterDown();
    intake.flapperOpen();
  }

  @Override
  public void execute() {
    intake.spinIn();
    uprighter.spinIn();
    gripper.spinIn();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
