package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Gripper;

public class OuttakeGamePiece extends CommandBase {

  Gripper gripper;

  public OuttakeGamePiece(Gripper gripperSubsystem) {
    gripper = gripperSubsystem;
    addRequirements(gripper);
  }

  @Override
  public void initialize() {
    if (RobotContainer.readyForCone)
      gripper.openPincher();
  }

  @Override
  public void execute() {
    gripper.spinOut();
  }

  @Override
  public void end(boolean interrupted) {
    gripper.stopSpinner();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
