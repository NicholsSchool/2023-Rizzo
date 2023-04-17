package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Deploys the intake and spins the intake motors inwards.
 */
public class DeployIntake extends CommandBase {

  private Intake intake;
  private Uprighter uprighter;

  public DeployIntake(Intake _intake, Uprighter _uprighter) {
    intake = _intake;
    uprighter = _uprighter;
    addRequirements(intake, uprighter);
  }

  @Override
  public void initialize() {
    intake.lower();
    intake.open();
  }

  @Override
  public void execute() {
    intake.spinIn();
    uprighter.spinIn();
  }

  @Override
  public void end(boolean interrupted) {
    intake.raise();
    intake.stop();
    uprighter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
