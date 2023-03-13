package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uprighter;

public class SpinEverythingOut extends CommandBase {

  private Intake intake;
  private Uprighter uprighter;
  private Gripper gripper;

  public SpinEverythingOut(Intake intakeSubsystem, Uprighter uprighterSubsystem, Gripper gripperSubsystem) {
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
    intake.spinOut();
    uprighter.spinOut();
    gripper.spinOut();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    uprighter.stop();
    gripper.stopSpinner();
    intake.lifterUp();
    intake.flapperClose();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
