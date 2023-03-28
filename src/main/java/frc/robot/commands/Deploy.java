package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.*;

/**
 * Deploys the intake down and spins the intake motors inwards.
 */
public class Deploy extends CommandBase {

  private Intake intake;
  private Uprighter uprighter;
  private Gripper gripper;
  XboxController rumbler;

  public Deploy(Intake _intake, Uprighter _uprighter, Gripper _gripper, XboxController _rumbler) {

    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    rumbler = _rumbler;

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
    if (!gripper.isPressed()) {
      rumbler.setRumble(RumbleType.kBothRumble, 0.75);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.raise();
    intake.open();
    intake.stop();
    uprighter.stop();
    gripper.stop();
    rumbler.setRumble(RumbleType.kBothRumble, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
