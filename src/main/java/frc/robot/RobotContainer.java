package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;

import frc.robot.Constants.SwerveDriveConstants;
import static frc.robot.Constants.ArmConstants.*;

public class RobotContainer {

  // Create subsystems
  private final SwerveDrive swerveDrive;
  private final Gripper gripper;
  // private final Arm arm;
  private final CBArm cbarm;
  private final Intake intake;
  private final Uprighter uprighter;
  Compressor compressor;

  // OI controllers
  CommandXboxController driverOI;
  CommandXboxController operatorOI;

  // Autonomous Commands
  // private final DefaultAuto defaultAuto;

  // Used for determining if gripper is picking up cone or cube
  public static boolean readyForCube = false;

  /** Robot Container Constructor. */
  public RobotContainer() {

    // Instantiate all subsystems
    swerveDrive = new SwerveDrive();
    gripper = new Gripper();
    // arm = new Arm();
    cbarm = new CBArm();
    intake = new Intake();
    uprighter = new Uprighter();
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    // Instantiate all autonomous commands
    // defaultAuto = new DefaultAuto(swerveDrive);

    // Instantiate all OI controllers
    driverOI = new CommandXboxController(1);
    operatorOI = new CommandXboxController(0);

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Define all button() to command() mappings. */
  private void configureButtonBindings() {

    // ########################################################
    // ################# DRIVER OI CONTROLLER #################
    // ########################################################

    // DRIVER Left & Right Stick: Field relative translational/rotational movement.
    // working
    swerveDrive.setDefaultCommand(
        new RunCommand(
            () -> swerveDrive.drive(
                -MathUtil.applyDeadband(driverOI.getLeftY(), 0.07),
                -MathUtil.applyDeadband(driverOI.getLeftX(), 0.07),
                -MathUtil.applyDeadband(driverOI.getRightX(), 0.07),
                true),
            swerveDrive));

    // DRIVER Left Trigger: (WH) Switch to virtual high gear.
    // working
    driverOI.leftTrigger(0.25)
        .onTrue(new InstantCommand(() -> swerveDrive.setVirtualHighGear()))
        .onFalse(new InstantCommand(() -> swerveDrive.setVirtualLowGear()));

    // DRIVER Right Trigger: (WH) Deploy intake when pressed and spin motors.
    // working
    driverOI.rightTrigger().whileTrue(new IntakeDeploy(intake, uprighter, gripper));
    driverOI.rightTrigger().onFalse(new IntakeRetract(intake, uprighter, gripper));

    // DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
    // not working
    driverOI.povRight().whileTrue(new InstantCommand(() -> swerveDrive.drive(0.0, -0.5, 0, false)).withTimeout(0.25));
    driverOI.povLeft().whileTrue(new InstantCommand(() -> swerveDrive.drive(0.0, 0.5, 0, false)).withTimeout(0.25));
    driverOI.povUp().whileTrue(new InstantCommand(() -> swerveDrive.drive(0.5, 0.0, 0, false)).withTimeout(0.25));
    driverOI.povDown().whileTrue(new InstantCommand(() -> swerveDrive.drive(-0.5, 0.0, 0, false)).withTimeout(0.25));

    // DRIVER Start Button: Reset the robot's field oriented forward position.
    // working
    driverOI.start().whileTrue(new RunCommand(() -> swerveDrive.resetFieldOrientedGyro(), swerveDrive));

    // DRIVER Back Button: While held, defensive X position and prevent driving.
    // NOT working (Camden)
    driverOI.x().whileTrue(new RunCommand(() -> swerveDrive.setX(), swerveDrive));

    // ########################################################
    // ################ OPERATOR OI CONTROLLER ################
    // ########################################################

    // OPERATOR Left Stick: Direct control over the Arm.
    cbarm.setDefaultCommand(new RunCommand(() -> cbarm.move(operatorOI.getLeftY()), cbarm));

    operatorOI.x().onTrue(new InstantCommand(() -> cbarm.setPositionUsingPID(HOME_POSITION)));
    operatorOI.y().whileTrue(new InstantCommand(() -> cbarm.setPositionUsingPID(20.0)));
    operatorOI.b().onTrue(new InstantCommand(() -> cbarm.setPositionUsingPID(SCORING_POSITION)));
    operatorOI.a().onTrue(new InstantCommand(() -> cbarm.setPositionUsingPID(GROUND_POSITION)));

    // OPERATOR Right Stick: Direct control over the Uprighter.
    // working
    uprighter.setDefaultCommand(new RunCommand(
        () -> uprighter.spin(-MathUtil.applyDeadband(operatorOI.getRightY(), 0.07)), uprighter));

    // OPERATOR Right Trigger: Release game object from Grabber.
    // working
    operatorOI.rightTrigger().whileTrue(new IntakeExtract(intake, uprighter, gripper));
    operatorOI.rightTrigger().onFalse(new IntakeRetract(intake, uprighter, gripper));

    // OPERATOR POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the field.
    // not working (Cameden)
    operatorOI.povRight().whileTrue(new InstantCommand(() -> swerveDrive.drive(-0.5, 0.0, 0, true)).withTimeout(0.25));
    operatorOI.povLeft().whileTrue(new InstantCommand(() -> swerveDrive.drive(0.5, 0.0, 0, true)).withTimeout(0.25));
    operatorOI.povUp().whileTrue(new InstantCommand(() -> swerveDrive.drive(0.0, 0.5, 0, true)).withTimeout(0.25));
    operatorOI.povDown().whileTrue(new InstantCommand(() -> swerveDrive.drive(0.0, -0.5, 0, true)).withTimeout(0.25));

  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        SwerveDriveConstants.MAX_METERS_PER_SECOND,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(4.0, 0)),
        new Pose2d(4.5, 1.5, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        swerveDrive::getPose, // Functional interface to feed supplier
        SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        swerveDrive::setModuleStates,
        swerveDrive);

    // Reset odometry to the starting pose of the trajectory.
    swerveDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> swerveDrive.drive(0, 0, 0, false));
  }

}
