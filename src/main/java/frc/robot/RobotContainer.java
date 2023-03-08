package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import static frc.robot.Constants.SwerveDriveConstants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

public class RobotContainer {

  // subsystems
  private final SwerveDrive robotSwerveDrive = new SwerveDrive();

  // OI controllers
  XboxController driverXbox = new XboxController(0);
  XboxController operatorXbox = new XboxController(1);

  /** Robot Container Constructor. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    robotSwerveDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotSwerveDrive.drive(
                -MathUtil.applyDeadband(driverXbox.getLeftY(), 0.07),
                -MathUtil.applyDeadband(driverXbox.getLeftX(), 0.07),
                -MathUtil.applyDeadband(driverXbox.getRightX(), 0.07),
                true),
            robotSwerveDrive));
  }

  /** Define all button() to command() mappings. */
  private void configureButtonBindings() {
    new JoystickButton(driverXbox, Button.kX.value)
        .whileTrue(new RunCommand(() -> robotSwerveDrive.setX(), robotSwerveDrive));

    new JoystickButton(driverXbox, Button.kY.value)
        .whileTrue(new RunCommand(() -> robotSwerveDrive.resetGyro(), robotSwerveDrive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
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
        robotSwerveDrive::getPose, // Functional interface to feed supplier
        SWERVE_DRIVE_KINEMATICS,
        // Position controllers
        new PIDController(0.85, 0, 0),
        new PIDController(0.85, 0, 0),
        thetaController,
        robotSwerveDrive::setModuleStates,
        robotSwerveDrive);

    // Reset odometry to the starting pose of the trajectory.
    robotSwerveDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> robotSwerveDrive.drive(0, 0, 0, false));
  }

}
