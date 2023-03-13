package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uprighter;

public class DeployIntake extends CommandBase {

  private Intake intake;
  private Uprighter uprighter;

  public DeployIntake(Intake intakeSubsystem, Uprighter uprighterSubsystem) {
    intake = intakeSubsystem = intake;
    uprighter = uprighterSubsystem = uprighter;
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
