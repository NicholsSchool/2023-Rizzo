package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Deploys the intake down and spins the intake motors inwards.
 */
public class Deploy extends CommandBase {

  private Intake intake;
  private Uprighter uprighter;
  private Gripper gripper;

  public Deploy(Intake _intake, Uprighter _uprighter, Gripper _gripper) {

    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;

    addRequirements(intake, uprighter, gripper);
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
    gripper.spinIn();
  }

  @Override
  public void end(boolean interrupted) {
    intake.raise();
    intake.open();
    intake.stop();
    uprighter.stop();
    gripper.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
