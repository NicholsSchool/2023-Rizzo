package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class SpinGripper extends CommandBase {

  private Gripper gripper;
  XboxController driverRumbler;
  XboxController operatorRumbler;
  double startTime;
  boolean isRumbling;

  public SpinGripper(XboxController _driverRumbler, XboxController _operatorRumbler, Gripper _gripper) {
    driverRumbler = _driverRumbler;
    operatorRumbler = _operatorRumbler;
    gripper = _gripper;
    addRequirements(gripper);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    gripper.spin(-RobotContainer.operatorOI.getLeftY());
    if (!gripper.isPressed()) {
      operatorRumbler.setRumble(RumbleType.kBothRumble, 1.0);
      driverRumbler.setRumble(RumbleType.kBothRumble, 1.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    gripper.stop();
    operatorRumbler.setRumble(RumbleType.kBothRumble, 0.0);
    driverRumbler.setRumble(RumbleType.kBothRumble, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
