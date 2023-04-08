package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class BalanceRobot extends CommandBase {

  SwerveDrive swerveDrive;
  boolean isFacingChargeStation = false;

  private static final double ANGLE_OF_INCLINE = 12.5;
  private static final double ANGLE_OF_DECLINE = -6.25;

  private static double ANGLE_OF_INCLINE_SPEED = 0.33;
  private static double ANGLE_OF_DECLINE_SPEED = -0.25;

  public BalanceRobot(SwerveDrive _swerveDrive, boolean _isFacingChargeStation) {
    swerveDrive = _swerveDrive;
    isFacingChargeStation = _isFacingChargeStation;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Balancing", true);
  }

  @Override
  public void execute() {

    // double currentAngle = swerveDrive.getRoll();
    double currentAngle = swerveDrive.getPitch();

    System.out.println("Current Angle: " + currentAngle);

    // Note: Guessing at the angles here, will need to test.
    if (isFacingChargeStation && currentAngle > ANGLE_OF_INCLINE) {
      swerveDrive.drive(ANGLE_OF_INCLINE_SPEED, 0.0, 0.0, true);
    } else if (isFacingChargeStation && currentAngle < ANGLE_OF_DECLINE) {
      swerveDrive.drive(ANGLE_OF_DECLINE_SPEED, 0.0, 0.0, true);
    } else if (!isFacingChargeStation && currentAngle > ANGLE_OF_INCLINE) {
      swerveDrive.drive(-ANGLE_OF_INCLINE_SPEED, 0.0, 0.0, true);
    } else if (!isFacingChargeStation && currentAngle < ANGLE_OF_DECLINE) {
      swerveDrive.drive(-ANGLE_OF_DECLINE_SPEED, 0.0, 0.0, true);
    } else {
      // swerveDrive.drive(0.0, 0.0, 0.0, true);
      swerveDrive.setWheelsToXFormation();
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Balancing", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
