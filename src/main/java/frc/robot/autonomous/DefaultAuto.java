package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import static frc.robot.Constants.SwerveDriveConstants.*;
import java.util.List;

public class DefaultAuto {

  private static SwerveDrive swerveDrive;

  public DefaultAuto(SwerveDrive swerveDrive) {
    DefaultAuto.swerveDrive = swerveDrive;
  }

  public Command runAutoSequence() {

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        2.0,
        2.4)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(SWERVE_DRIVE_KINEMATICS);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, .75),
            new Translation2d(3, 0),
            new Translation2d(2, -0.75)),
        new Pose2d(0.64, -0.07, new Rotation2d(3.14)),
        config);

    var thetaController = new ProfiledPIDController(
        1, 0, 0, new TrapezoidProfile.Constraints(
            Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        swerveDrive::getPose, // Functional interface to feed supplier
        SWERVE_DRIVE_KINEMATICS,
        // Position controllers
        new PIDController(0.85, 0, 0),
        new PIDController(0.85, 0, 0),
        thetaController,
        swerveDrive::setModuleStates,
        swerveDrive);

    // Reset odometry to the starting pose of the trajectory.
    swerveDrive.resetOdometry(exampleTrajectory.getInitialPose());
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> swerveDrive.drive(0, 0, 0, false));
  }

}
