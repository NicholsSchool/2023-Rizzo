package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.*;

public class SpinGripper extends CommandBase {

  private Double speed;
  private Gripper gripper;
  XboxController driverRumbler;
  XboxController operatorRumbler;
  double startTime;
  boolean isRumbling;

  public SpinGripper(Double _speed, XboxController _driverRumbler, XboxController _operatorRumbler, Gripper _gripper) {
    speed = _speed;
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
    gripper.spin(speed);
    if (!gripper.isPressed()) {
      isRumbling = true;
      startTime = System.currentTimeMillis() / 1000;
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (isRumbling && (System.currentTimeMillis() / 1000 - startTime) < 2.0) {
      operatorRumbler.setRumble(RumbleType.kBothRumble, 1.0);
      driverRumbler.setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      isRumbling = false;
      operatorRumbler.setRumble(RumbleType.kBothRumble, 0.0);
      driverRumbler.setRumble(RumbleType.kBothRumble, 0.0);
    }
    return false;
  }
}
