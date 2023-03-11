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
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import static frc.robot.Constants.SwerveDriveConstants.*;

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

    // ################ DRIVER OI CONTROLLER CONFIGURATION ################

    // +DRIVER Left Stick: Translational movement relative to the field.
    // +DRIVER Right Stick: Rotational movement of the chassis along the X-axis.
    // DRIVER X Button: Rotate to -90 degree Yaw relative to the field.
    // DRIVER Y Button: Rotate to 0 degree Yaw relative to the field.
    // DRIVER B Button: Rotate to 90 degree Yaw relative to the field.
    // DRIVER A Button: Rotate to 180 degree Yaw relative to the field.
    // +DRIVER Left Trigger: While held, switch to virtual high gear.
    // DRIVER Right Trigger: While held, deploy and spin robot intake.
    // DRIVER Left Bumper: Evasive left robot action button.
    // DRIVER Right Bumper: Evasive right robot action button.
    // DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
    // +DRIVER Back Button: Reset the robot's field oriented forward position.
    // DRIVER Start Button: Toggle robot relative vs field orientated driving.

    // DRIVER Left Trigger: While held, switch to virtual high gear.
    driverOI.leftTrigger(0.25)
        .onTrue(new InstantCommand(() -> robotSwerveDrive.setVirtualHighGear()))
        .onFalse(new InstantCommand(() -> robotSwerveDrive.setVirtualLowGear()));

    // DRIVER Back Button: Reset the robot's field oriented forward position.
    driverOI.back().whileTrue(new RunCommand(() -> robotSwerveDrive.resetFieldOrientedGyro(), robotSwerveDrive));

    // Driver OI Controller Sample Mappings
    driverOI.a().onTrue(new InstantCommand(() -> System.out.println("Driver A")));
    driverOI.b().onTrue(new InstantCommand(() -> System.out.println("Driver B")));
    driverOI.x().onTrue(new InstantCommand(() -> System.out.println("Driver X")));
    driverOI.y().onTrue(new InstantCommand(() -> System.out.println("Driver Y")));
    driverOI.povLeft().onTrue(new InstantCommand(() -> System.out.println("Driver POV Left")));
    driverOI.povRight().onTrue(new InstantCommand(() -> System.out.println("Driver POV Right")));
    driverOI.povUp().onTrue(new InstantCommand(() -> System.out.println("Driver POV Up")));
    driverOI.povDown().onTrue(new InstantCommand(() -> System.out.println("Driver POV Down")));
    driverOI.leftTrigger().onTrue(new InstantCommand(() -> System.out.println("Driver Left Trigger")));
    driverOI.rightTrigger().onTrue(new InstantCommand(() -> System.out.println("Driver Right Trigger")));
    driverOI.leftBumper().onTrue(new InstantCommand(() -> System.out.println("Driver Left Bumper")));
    driverOI.rightBumper().onTrue(new InstantCommand(() -> System.out.println("Driver Right Bumper")));
    driverOI.back().onTrue(new InstantCommand(() -> System.out.println("Driver Back")));
    driverOI.start().onTrue(new InstantCommand(() -> System.out.println("Driver Start")));

    // ################ OPERATOR OI CONTROLLER CONFIGURATION ################

    // OPERATOR Left Stick: Direct control over the Arm. Overrides arm locks.
    // OPERATOR Right Stick: (not used)
    // OPERATOR X Button: Go to Arm position #1 and lock.
    // OPERATOR Y Button: Go to Arm position #2 and lock.
    // OPERATOR B Button: Go to Arm position #3 and lock.
    // OPERATOR A Button: Go to Arm position #4 and lock.
    // OPERATOR Left Bumper: Cycle through Grid numbers 1-9 for object placement.
    // OPERATOR Right Bumper: Cycle through Tier numbers 1-3 for object placement.
    // OPERATOR Left Trigger: While held, auto pick up game object using ML/AI.
    // OPERATOR Right Trigger: Release game object from Grabber.
    // OPERATOR POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the field.
    // ~OPERATOR Back Button: Toggle defensive X position and prevent driving.
    // OPERATOR Start Button: Cycle out all intake and grabber motors.

    // OPERATOR Back Button: Toggle defensive X position and prevent driving.
    operatorOI.back().whileTrue(new RunCommand(() -> robotSwerveDrive.setX(), robotSwerveDrive));

    // Operator OI Controller Sample Mappings
    operatorOI.a().onTrue(new InstantCommand(() -> System.out.println("Operator A")));
    operatorOI.b().onTrue(new InstantCommand(() -> System.out.println("Operator B")));
    operatorOI.x().onTrue(new InstantCommand(() -> System.out.println("Operator X")));
    operatorOI.y().onTrue(new InstantCommand(() -> System.out.println("Operator Y")));
    operatorOI.povLeft().onTrue(new InstantCommand(() -> System.out.println("Operator POV Left")));
    operatorOI.povRight().onTrue(new InstantCommand(() -> System.out.println("Operator POV Right")));
    operatorOI.povUp().onTrue(new InstantCommand(() -> System.out.println("Operator POV Up")));
    operatorOI.povDown().onTrue(new InstantCommand(() -> System.out.println("Operator POV Down")));
    operatorOI.leftTrigger().onTrue(new InstantCommand(() -> System.out.println("Operator Left Trigger")));
    operatorOI.rightTrigger().onTrue(new InstantCommand(() -> System.out.println("Operator Right Trigger")));
    operatorOI.leftBumper().onTrue(new InstantCommand(() -> System.out.println("Operator Left Bumper")));
    operatorOI.rightBumper().onTrue(new InstantCommand(() -> System.out.println("Operator Right Bumper")));
    operatorOI.back().onTrue(new InstantCommand(() -> System.out.println("Operator Back")));
    operatorOI.start().onTrue(new InstantCommand(() -> System.out.println("Operator Start")));
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
