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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import static frc.robot.Constants.SwerveDriveConstants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;

public class RobotContainer {

  // subsystems
  private final SwerveDrive robotSwerveDrive = new SwerveDrive();

  // OI controllers
  CommandXboxController driverOI = new CommandXboxController(0);
  CommandXboxController operatorOI = new CommandXboxController(1);

  /** Robot Container Constructor. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    robotSwerveDrive.setDefaultCommand(
        new RunCommand(
            () -> robotSwerveDrive.drive(
                -MathUtil.applyDeadband(driverOI.getLeftY(), 0.07),
                -MathUtil.applyDeadband(driverOI.getLeftX(), 0.07),
                -MathUtil.applyDeadband(driverOI.getRightX(), 0.07),
                true),
            robotSwerveDrive));
  }

  /** Define all button() to command() mappings. */
  private void configureButtonBindings() {

    // DRIVER X Button: Set swerve drive to X position.
    driverOI.x()
        .whileTrue(new RunCommand(() -> robotSwerveDrive.setX(), robotSwerveDrive));

    // DRIVER Left Trigger: Shift between high and low gear.
    driverOI.leftTrigger(0.25)
        .onTrue(new InstantCommand(() -> robotSwerveDrive.setVirtualHighGear()))
        .onFalse(new InstantCommand(() -> robotSwerveDrive.setVirtualLowGear()));

    // DRIVER Y Button: Reset field oriented gyro.
    driverOI.y()
        .whileTrue(new RunCommand(() -> robotSwerveDrive.resetFieldOrientedGyro(), robotSwerveDrive));

    // Driver OI Controller Sample Mappings
    driverOI.a().onTrue(new InstantCommand(() -> System.out.println("Driver A")));
    driverOI.b().onTrue(new InstantCommand(() -> System.out.println("Driver B")));
    driverOI.x().onTrue(new InstantCommand(() -> System.out.println("Driver X")));
    driverOI.y().onTrue(new InstantCommand(() -> System.out.println("Driver Y")));
    driverOI.start().onTrue(new InstantCommand(() -> System.out.println("Driver Start")));
    driverOI.back().onTrue(new InstantCommand(() -> System.out.println("Driver Back")));
    driverOI.leftBumper().onTrue(new InstantCommand(() -> System.out.println("Driver Left Bumper")));
    driverOI.rightBumper().onTrue(new InstantCommand(() -> System.out.println("Driver Right Bumper")));
    driverOI.povLeft().onTrue(new InstantCommand(() -> System.out.println("Driver POV Left")));
    driverOI.povUpLeft().onTrue(new InstantCommand(() -> System.out.println("Driver POV Up Left")));
    driverOI.povRight().onTrue(new InstantCommand(() -> System.out.println("Driver POV Right")));
    driverOI.povUpRight().onTrue(new InstantCommand(() -> System.out.println("Driver POV Up Right")));
    driverOI.povUp().onTrue(new InstantCommand(() -> System.out.println("Driver POV Up")));
    driverOI.povDownRight().onTrue(new InstantCommand(() -> System.out.println("Driver POV Down Right")));
    driverOI.povDownLeft().onTrue(new InstantCommand(() -> System.out.println("Driver POV Down Left")));
    driverOI.povDown().onTrue(new InstantCommand(() -> System.out.println("Driver POV Down")));
    driverOI.leftTrigger().onTrue(new InstantCommand(() -> System.out.println("Driver Left Trigger")));
    driverOI.rightTrigger().onTrue(new InstantCommand(() -> System.out.println("Driver Right Trigger")));

    // Operator OI Controller Sample Mappings
    operatorOI.a().onTrue(new InstantCommand(() -> System.out.println("Operator A")));
    operatorOI.b().onTrue(new InstantCommand(() -> System.out.println("Operator B")));
    operatorOI.x().onTrue(new InstantCommand(() -> System.out.println("Operator X")));
    operatorOI.y().onTrue(new InstantCommand(() -> System.out.println("Operator Y")));
    operatorOI.start().onTrue(new InstantCommand(() -> System.out.println("Operator Start")));
    operatorOI.back().onTrue(new InstantCommand(() -> System.out.println("Operator Back")));
    operatorOI.leftBumper().onTrue(new InstantCommand(() -> System.out.println("Operator Left Bumper")));
    operatorOI.rightBumper().onTrue(new InstantCommand(() -> System.out.println("Operator Right Bumper")));
    operatorOI.povLeft().onTrue(new InstantCommand(() -> System.out.println("Operator POV Left")));
    operatorOI.povUpLeft().onTrue(new InstantCommand(() -> System.out.println("Operator POV Up Left")));
    operatorOI.povRight().onTrue(new InstantCommand(() -> System.out.println("Operator POV Right")));
    operatorOI.povUpRight().onTrue(new InstantCommand(() -> System.out.println("Operator POV Up Right")));
    operatorOI.povUp().onTrue(new InstantCommand(() -> System.out.println("Operator POV Up")));
    operatorOI.povDownRight().onTrue(new InstantCommand(() -> System.out.println("Operator POV Down Right")));
    operatorOI.povDownLeft().onTrue(new InstantCommand(() -> System.out.println("Operator POV Down Left")));
    operatorOI.povDown().onTrue(new InstantCommand(() -> System.out.println("Operator POV Down")));
    operatorOI.leftTrigger().onTrue(new InstantCommand(() -> System.out.println("Operator Left Trigger")));
    operatorOI.rightTrigger().onTrue(new InstantCommand(() -> System.out.println("Operator Right Trigger")));
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
