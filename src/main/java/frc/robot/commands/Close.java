package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.*;

public class Close extends CommandBase {

  private Gripper gripper;
  XboxController driverRumbler;
  XboxController operatorRumbler;
  double startTime;

  public Close(Gripper _gripper, XboxController _driverRumbler, XboxController _operatorRumbler) {
    gripper = _gripper;
    driverRumbler = _driverRumbler;
    operatorRumbler = _operatorRumbler;
    addRequirements(gripper);
  }

  @Override
  public void initialize() {
    gripper.close();
    startTime = System.currentTimeMillis() / 1000;
  }

  @Override
  public void execute() {
    if (!gripper.isPressed()) {
      operatorRumbler.setRumble(RumbleType.kBothRumble, 1.00);
      driverRumbler.setRumble(RumbleType.kBothRumble, 1.00);
    }
  }

  @Override
  public void end(boolean interrupted) {
    operatorRumbler.setRumble(RumbleType.kBothRumble, 0.0);
    driverRumbler.setRumble(RumbleType.kBothRumble, 0.0);
  }

  @Override
  public boolean isFinished() {
    if ((System.currentTimeMillis() / 1000 - startTime) < 2.0) {
      return false;
    } else {
      return true;
    }
  }
}
