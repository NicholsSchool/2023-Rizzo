package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uprighter;

/**
 * Deploys the intake and uprighter.
 */
public class DeployIntake extends CommandBase {

  private Intake intake;
  private Uprighter uprighter;

  /**
   * Creates a new DeployIntake.
   * 
   * @param intakeSub
   * @param uprighterSub
   */
  public DeployIntake(Intake intakeSub, Uprighter uprighterSub) {
    intake = intakeSub;
    uprighter = uprighterSub;
    addRequirements(intake, uprighter);
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
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
