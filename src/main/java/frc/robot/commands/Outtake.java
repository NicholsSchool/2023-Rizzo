package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Outtake a Cube from the intake using the intake/uprighter/gripper motors.
 */
public class Outtake extends CommandBase {

  Intake intake;

  public Outtake(Intake _intake ) {

    intake = _intake;
    

    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.spinOut();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
