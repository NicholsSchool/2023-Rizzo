package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.AutoDistanceConstants.*;

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
        new OuttakeCube(intake, uprighter, gripper, OUTTAKE_HIGH_SPEED).withTimeout(0.5),
        new BalanceRobot(swerveDrive, APRILTAG_TO_END_OF_COMMUNITY_METERS).withTimeout(3.5),
        new RotateRobot(swerveDrive, 0.0).withTimeout(0.5),
        new BalanceRobot(swerveDrive, 1.5).withTimeout(3.0),
        new RotateRobot(swerveDrive, 0.0).withTimeout(0.5),
        new BalanceRobot(swerveDrive, APRILTAG_TO_CHARGE_STATION_METERS).withTimeout(6.9),
        new InstantCommand(() -> swerveDrive.setGyroAngleAdjustment(180.0)));

  }

}
