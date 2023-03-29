package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.Deploy;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Uprighter;

public class AutoTest extends SequentialCommandGroup {
  /** Creates a new AutoTest. */
  public AutoTest(SwerveDrive swerveDrive, Intake intake, Uprighter uprighter, Gripper gripper, Arm arm) {

    // double pTheta = AutoConstants.kPThetaController;

    TrajectoryConfig config = new TrajectoryConfig(Math.PI, Math.PI)
        .setKinematics(SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS);

    Trajectory driveToGamePiece = TrajectoryGenerator.generateTrajectory(
        // Zero the starting pose of the trajectory.
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(

            new Translation2d(1, 0.1)),
        // Final X/Y position in meters and rotation in radians.
        new Pose2d(1.5, 0, new Rotation2d(0)),
        config);

    Trajectory backUpOneMeter = TrajectoryGenerator.generateTrajectory(
        // Zero the starting pose of the trajectory.
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(-0.5, 0.1)),
        // Final X/Y position in meters and rotation in radians.
        new Pose2d(-1, 0, new Rotation2d(0)),
        config);

    Trajectory driveToChargeStation = TrajectoryGenerator.generateTrajectory(
        // Zero the starting pose of the trajectory.
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, -0.5)),
        // Final X/Y position in meters and rotation in radians.
        new Pose2d(1, -1, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(1.0, 0, 0, new Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // System.out.println("Angle: " + swerveDrive.getHeading());
    System.out.println("I AM RUNNING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    addRequirements(swerveDrive, intake, gripper, arm, uprighter);
    addCommands(

        // TEST EVERYTHING INDIVIDUALLY AND THEN CHAIN TOGETHER

        // Arm stuff
        // Backup a little
        // Turn
        // Forward
        // Forward to pickup game piece
        // Turn
        // Drive to charge station
        // Shoot cube

        // Test
        new RunCommand(() -> gripper.close(), intake).withTimeout(2), // Close gripper on cone
        // new InstantCommand(() -> arm.setTargetPosition(ArmConstants.POSITION_01)), // will it move off of 0.0 ???
        // new InstantCommand(() -> arm.setTargetPosition(ArmConstants.POSITION_02)),
        // new InstantCommand(() -> arm.setTargetPosition(ArmConstants.POSITION_03)),
        // new InstantCommand(() -> arm.setTargetPosition(ArmConstants.POSITION_04)),
        // new InstantCommand(() -> arm.setTargetPosition(ArmConstants.POSITION_05)),
        // new InstantCommand(() -> arm.setTargetPosition(ArmConstants.POSITION_06)), // Bring arm to scoring position
        new RunCommand(() -> gripper.open(), intake) // Open gripper
       // new InstantCommand(() -> arm.setTargetPosition(ArmConstants.POSITION_00)) // Might change. Depends on if we
                                                                                  // trust the slowing down
    // new InstantCommand(() -> arm.setTargetPosition(ArmConstants.POSITION_00)) //
    // Bring arm back to home position

    // //Test
    // //Drive backward one meter
    // new SwerveControllerCommand( backUpOneMeter, swerveDrive::getPose,
    // SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS,
    // new PIDController(1.0, 0, 0), new PIDController(1.0, 0, 0), thetaController,
    // swerveDrive::setModuleStates, swerveDrive),

    // //new Rotate()

    // //Test
    // //Drive to pick up cube
    // new SwerveControllerCommand( driveToGamePiece, swerveDrive::getPose,
    // SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS,
    // new PIDController(1.0, 0, 0), new PIDController(1.0, 0, 0), thetaController,
    // swerveDrive::setModuleStates, swerveDrive).raceWith(new Deploy(intake,
    // uprighter, gripper).withTimeout(1.5) ),

    // //new Rotate()

    // //Test
    // //Drive to charge station
    // new SwerveControllerCommand( driveToChargeStation, swerveDrive::getPose,
    // SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS,
    // new PIDController(1.0, 0, 0), new PIDController(1.0, 0, 0), thetaController,
    // swerveDrive::setModuleStates, swerveDrive)

    // //new ShootCube()

    );
  }
}
