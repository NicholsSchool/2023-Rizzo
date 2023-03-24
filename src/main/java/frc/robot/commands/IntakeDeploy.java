package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uprighter;

/**
 * Deploys the intake and uprighter.
 */
public class IntakeDeploy extends CommandBase {

  private Intake intake;
  private Uprighter uprighter;
  private Gripper gripper;

  public IntakeDeploy(Intake intakeSubsystem, Uprighter uprighterSubsystem, Gripper gripperSubsystem) {
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

    intake.lifterUp();
    intake.flapperClose();
    intake.stop();
    uprighter.stop();
    gripper.stopSpinner();;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
