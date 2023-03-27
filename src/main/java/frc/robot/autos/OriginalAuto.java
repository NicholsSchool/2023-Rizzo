package frc.robot.autos;

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
import static frc.robot.Constants.SwerveDriveConstants.*;

public class OriginalAuto extends SequentialCommandGroup {

  SwerveDrive swerveDrive;

  public OriginalAuto(SwerveDrive _swerveDrive) {

    swerveDrive = _swerveDrive;

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

    // Run path following command, then stop at the end.
    addCommands(swerveCC.andThen(() -> swerveDrive.drive(0, 0, 0, false)));

  }

}
