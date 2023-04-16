package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.AutoDistanceConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class ChargingCommunity extends SequentialCommandGroup {
  SwerveDrive swerveDrive;
  Intake intake;
  Uprighter uprighter;
  Gripper gripper;
  Arm arm;

  public ChargingCommunity(SwerveDrive _swerveDrive, Intake _intake, Uprighter _uprighter, Gripper _gripper, Arm _arm) {

    swerveDrive = _swerveDrive;
    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    arm = _arm;

    addRequirements(swerveDrive, intake, gripper, arm, uprighter);

    addCommands(new RunCommand(() -> uprighter.spinOut(), intake).withTimeout(0.5),
        new InstantCommand(() -> uprighter.stop(), intake),
        new OuttakeCube(intake, uprighter, gripper, IntakeConstants.OUTTAKE_HIGH_SPEED).withTimeout(0.5));
    // What happens if we don't see the AprilTag?
    addCommands(
        new BalanceRobot(_swerveDrive, AutoDistanceConstants.APRILTAG_TO_END_OF_COMMUNITY_METERS).withTimeout(5));
    addCommands(new RotateRobot(_swerveDrive, 0.0).withTimeout(1));
    addCommands(new BalanceRobot(_swerveDrive, AutoDistanceConstants.APRILTAG_TO_CHARGE_STATION_METERS).withTimeout(8));

  }
}
