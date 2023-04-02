package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Constants.SwerveDriveConstants.*;

public class Electric extends SequentialCommandGroup {

  SwerveDrive swerveDrive;
  Intake intake;
  Uprighter uprighter;
  Gripper gripper;
  Arm arm;

  public Electric(SwerveDrive _swerveDrive, Intake _intake, Uprighter _uprighter, Gripper _gripper, Arm _arm) {

    swerveDrive = _swerveDrive;
    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    arm = _arm;

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Math.PI, Math.PI).setKinematics(SWERVE_DRIVE_KINEMATICS);

    // All X/Y positions are relative to the robot. Auto is not field orientated!
    // Positive X is forward, positive Y is left, positive rotation is clockwise.
    Trajectory trajectory01 = TrajectoryGenerator.generateTrajectory(
        // Zero the starting pose of the trajectory.
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(-0.25, -0.25)
        // Add interior waypoints to the list above.
        ),
        // Final X/Y position in meters and rotation in radians.
        new Pose2d(-0.5, 0.0, new Rotation2d(0)), config);

    Trajectory trajectory02 = TrajectoryGenerator.generateTrajectory(
        // Zero the starting pose of the trajectory.
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        List.of(
            new Translation2d(1.25, -0.25)),
        // Final X/Y position in meters and rotation in radians.
        new Pose2d(4.15, -0.25, new Rotation2d(Math.PI)),
        config);

    // Create a PID controller for the robot's translation and rotation.
    var thetaController = new ProfiledPIDController(1.0, 0, 0, new Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand back0point5meters = new SwerveControllerCommand(
        trajectory01,
        swerveDrive::getPose,
        SWERVE_DRIVE_KINEMATICS,
        new PIDController(1.0, 0, 0),
        new PIDController(1.0, 0, 0),
        thetaController,
        swerveDrive::setModuleStates,
        swerveDrive);

    SwerveControllerCommand forward4meters = new SwerveControllerCommand(
        trajectory02,
        swerveDrive::getPose,
        SWERVE_DRIVE_KINEMATICS,
        new PIDController(1.0, 0, 0),
        new PIDController(1.0, 0, 0),
        thetaController,
        swerveDrive::setModuleStates,
        swerveDrive);

    // Reset odometry to the starting pose of the trajectory.
    swerveDrive.resetOdometry(trajectory01.getInitialPose());

    // Add all the requirements
    addRequirements(swerveDrive, intake, gripper, arm, uprighter);

    // Run path following command, then stop at the end.

    // Question: Can I spin the robot in place using the trajectory planner?
    // Answer: 45 degres points up and to the left

    addCommands(
        // 1.08 seconds - Initial shooting cube is optimal.
        new RunCommand(() -> uprighter.spinOut(), intake).withTimeout(0.65),
        new RunCommand(() -> uprighter.stop(), intake).withTimeout(0.0),
        new RunCommand(() -> intake.spinOut(), intake).withTimeout(0.43),
        new InstantCommand(() -> intake.stop(), intake));

    // Swerve Drive Auto
    addCommands(back0point5meters);

    // Move -90 degrees
    addCommands(new RotateRobot(swerveDrive, 90.0).withTimeout(0.88));

    // Move -90 degress and reset gyro
    addCommands(new RotateRobot(swerveDrive, 180.0).withTimeout(0.88)
        .andThen(() -> swerveDrive.resetGyro(), swerveDrive));

    addCommands(forward4meters);

  }

}
