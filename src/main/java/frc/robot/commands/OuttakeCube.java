package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Outtake a Cube from the intake using the intake/uprighter/gripper motors.
 */
public class OuttakeCube extends CommandBase {

  Intake intake;
  Uprighter uprighter;
  Gripper gripper;
  Double speed;

  public OuttakeCube(Intake _intake, Uprighter _uprighter, Gripper _gripper, Double _speed) {

    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    speed = _speed;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.spin(speed);
    uprighter.spinOut();
    gripper.spinOut();
  }

  @Override
  public void end(boolean interrupted) {
    intake.raise();
    intake.stop();
    uprighter.stop();
    gripper.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
