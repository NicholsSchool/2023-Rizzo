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

public class Test01Outtake extends SequentialCommandGroup {

  SwerveDrive swerveDrive;
  Intake intake;
  Uprighter uprighter;
  Gripper gripper;
  Arm arm;

  public Test01Outtake(SwerveDrive _swerveDrive, Intake _intake, Uprighter _uprighter, Gripper _gripper, Arm _arm) {

    swerveDrive = _swerveDrive;
    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    arm = _arm;

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Math.PI, Math.PI).setKinematics(SWERVE_DRIVE_KINEMATICS);

    // All X/Y positions are relative to the robot. Auto is not field orientated!
    // Positive X is forward, positive Y is left, positive rotation is clockwise.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Zero the starting pose of the trajectory.
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(4.0, 0)
        // Add interior waypoints to the list above.
        ),
        // Final X/Y position in meters and rotation in radians.
        new Pose2d(4.5, 1.5, new Rotation2d(0)), config);

    // Create a PID controller for the robot's translation and rotation.
    var thetaController = new ProfiledPIDController(1.0, 0, 0, new Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveCC = new SwerveControllerCommand(
        trajectory,
        swerveDrive::getPose,
        SWERVE_DRIVE_KINEMATICS,
        new PIDController(1.0, 0, 0),
        new PIDController(1.0, 0, 0),
        thetaController,
        swerveDrive::setModuleStates,
        swerveDrive);

    // Reset odometry to the starting pose of the trajectory.
    swerveDrive.resetOdometry(trajectory.getInitialPose());

    // Add all the requirements
    addRequirements(swerveDrive, intake, gripper, arm, uprighter);

    // Run path following command, then stop at the end.
    // addCommands(swerveCC.andThen(() -> swerveDrive.drive(0, 0, 0, false)));

    // Question: Will Daniel's code run even when we instatiate the swervedrive?
    // Answer: It works now that we call the right function
    addCommands(
        new PrintCommand("IS THIS WORKING?????????????????"),
        new RunCommand(() -> intake.close(), intake).withTimeout(0.1),
        new RunCommand(() -> intake.lower(), intake).withTimeout(1.0),
        new RunCommand(() -> intake.raise(), intake).withTimeout(1.5),
        new RunCommand(() -> intake.spinOut(), intake).withTimeout(2),
        new InstantCommand(() -> intake.stop(), intake)

    );
  }

}
