package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Retracts the intake up and stops all intake motors.
 */
public class Retract extends CommandBase {

  private Intake intake;
  private Uprighter uprighter;
  private Gripper gripper;

  public Retract(Intake _intake, Uprighter _uprighter, Gripper _gripper) {

    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;

    addRequirements(intake, uprighter, gripper);
  }

  @Override
  public void initialize() {
    intake.raise();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    gripper.stop();
    intake.stop();
    uprighter.stop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
