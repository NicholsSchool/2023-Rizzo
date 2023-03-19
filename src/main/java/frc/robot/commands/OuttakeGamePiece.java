package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class OuttakeGamePiece extends CommandBase {

  Intake intake;
  Uprighter uprighter;
  Gripper gripper;

  public OuttakeGamePiece(Intake _intake, Uprighter _uprighter, Gripper _gripper) {
    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    addRequirements(intake, uprighter, gripper);
  }

  @Override
  public void initialize() {
    // if (RobotContainer.readyForCone)
    // gripper.openPincher();
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
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
