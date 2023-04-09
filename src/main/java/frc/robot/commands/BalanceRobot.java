package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class BalanceRobot extends CommandBase {

  SwerveDrive swerveDrive;
  boolean isFacingChargeStation = false;

  boolean isAtStopPoint = false;

  private static final double ANGLE_OF_INCLINE = 12.5;
  private static final double ANGLE_OF_DECLINE = -6.25;

  private static double ANGLE_OF_INCLINE_SPEED = 0.7;
  private static double ANGLE_OF_DECLINE_SPEED = -0.33;

  public BalanceRobot(SwerveDrive _swerveDrive, boolean _isFacingChargeStation) {
    swerveDrive = _swerveDrive;
    isFacingChargeStation = _isFacingChargeStation;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    System.out.println("!!!!!!!!!!!! STARTING BALANCE ROUTINE !!!!!!!!!!!!!!");
  }

  @Override
  public void execute() {

    // double currentAngle = swerveDrive.getRoll();
    double currentAngle = swerveDrive.getPitch();

    PIDController pid = new PIDController(1.322, 0.0, 0);
    double power = pid.calculate(currentAngle, 0);

    // pid.setTolerance(5, 10);
    // pid.atSetpoint();

    double pow = -(power / 100) * 1.023;

    if (currentAngle < -2.85 && !isAtStopPoint) {
      swerveDrive.drive(pow, 0.0, 0.0, true);
      System.out.println("!!!!!!!! BALANCING: INCLINE AHEAD " + pow);
    } else if (currentAngle > -2.30 && !isAtStopPoint) {
      swerveDrive.drive(pow, 0.0, 0.0, true);
      System.out.println("!!!!!!!! BALANCING: DECLINE AHEAD " + pow);
    } else {
      isAtStopPoint = true;
      // swerveDrive.drive(0.0, 0.0, 0.0, true);
      swerveDrive.setWheelsToXFormation();
      System.out.println("!!!!!!!! BALANCING: STOPPING");
    }

    // // Note: Guessing at the angles here, will need to test.
    // if (isFacingChargeStation && currentAngle > ANGLE_OF_INCLINE) {
    // swerveDrive.drive(ANGLE_OF_INCLINE_SPEED, 0.0, 0.0, true);
    // } else if (isFacingChargeStation && currentAngle < ANGLE_OF_DECLINE) {
    // swerveDrive.drive(ANGLE_OF_DECLINE_SPEED, 0.0, 0.0, true);
    // } else if (!isFacingChargeStation && currentAngle > ANGLE_OF_INCLINE) {
    // System.out.println("!!!!!!!! BALANCING: INCLINE AHEAD " + currentAngle + "
    // >>> " + ANGLE_OF_INCLINE);
    // swerveDrive.drive(ANGLE_OF_INCLINE_SPEED, 0.0, 0.0, true);
    // } else if (!isFacingChargeStation && currentAngle < ANGLE_OF_DECLINE) {
    // System.out.println("!!!!!!!! BALANCING: DECLINE AHEAD " + currentAngle + "<<<
    // " + ANGLE_OF_DECLINE);
    // swerveDrive.drive(-ANGLE_OF_DECLINE_SPEED, 0.0, 0.0, true);
    // } else {
    // // swerveDrive.drive(0.0, 0.0, 0.0, true);
    // System.out.println("!!!!!!!! BALANCING: SET X POSITION");
    // swerveDrive.setWheelsToXFormation();
    // }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("!!!!!!!!!!!! ENDING BALANCE ROUTINE !!!!!!!!!!!!!!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
